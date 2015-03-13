;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-robohow-review-year3)

;;;
;;; Helper functions
;;;

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defmacro try-n-times (n &body body)
  `(cpl:with-retry-counters ((retry-count ,n))
     (cpl:with-failure-handling
         (((or cram-plan-failures:object-not-found
               cram-plan-failures:manipulation-failure
               cram-plan-failures:location-not-reached-failure) (f)
            (declare (ignore f))
            (cpl:do-retry retry-count
              (cpl:retry))))
       ,@body)))

(defun publish-pose (pose &optional (topic "/object"))
  (let ((adv (roslisp:advertise topic "geometry_msgs/PoseStamped")))
    (roslisp:publish adv (tf:pose-stamped->msg pose))))

(defun move-arms-up (&key allowed-collision-objects side ignore-collisions)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 0.5 1.3)
      (tf:euler->quaternion :ax 0));pi))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 -0.5 1.3)
      (tf:euler->quaternion :ax 0));pi))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects)

(defun ensure-arms-up (&optional (side (list :left :right)))
  (cond ((listp side)
         (dolist (s side)
           (ensure-arms-up s)))
        (t
         (let ((ignore-collisions nil))
           (cpl:with-failure-handling
               ((cram-plan-failures:manipulation-failure (f)
                  (declare (ignore f))
                  (setf ignore-collisions t)
                  (cpl:retry)))
             (move-arms-up :side side
                           :ignore-collisions ignore-collisions))))))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            'cram-designators:object
                            (append
                             `((desig-props:type desig-props:handle)
                               (desig-props:at
                                ,(a location `((desig-props:pose
                                                ,handle-pose)))))
                             (when grasp-type
                               `((desig-props:grasp-type ,grasp-type)))))
        collect handle-object))))

;;;
;;; Plan Macros
;;;

(defmacro perceive-a (object &key stationary (move-head t))
  `(cpl:with-failure-handling
       ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (ros-warn (longterm) "Object not found. Retrying.")
          (cpl:retry)))
     (cond (,stationary
            (let ((at (desig-prop-value ,object 'desig-props:at)))
              (when ,move-head
                (achieve `(cram-plan-library:looking-at ,(reference at))))
              (first (perceive-object
                      'cram-plan-library:currently-visible
                      ,object))))
           (t (cpl:with-failure-handling
                  ((cram-plan-failures:location-not-reached-failure (f)
                     (declare (ignore f))
                     (cpl:retry)))
                (perceive-object 'cram-plan-library:a ,object))))))

(defmacro pick-object (object &key stationary)
  `(cpl:with-retry-counters ((retry-task 0))
     (cpl:with-failure-handling
         ((cram-plan-failures:manipulation-failure (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ensure-arms-up)
              (cpl:retry)))
          (cram-plan-failures:location-not-reached-failure (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ros-warn (longterm) "Cannot reach location. Retrying.")
              (cpl:retry)))
          (cram-plan-failures:object-not-found (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ros-warn (longterm) "Object not found. Retrying.")
              (cpl:retry))))
       ,(cond (stationary
               `(achieve `(cram-plan-library:object-picked ,,object)))
              (t
               `(achieve `(cram-plan-library:object-in-hand ,,object)))))))

(defmacro place-object (object location &key stationary)
  `(cpl:with-failure-handling
       ((cram-plan-failures:manipulation-pose-unreachable (f)
          (declare (ignore f))
          (cram-plan-library::retry-with-updated-location
           ,location (next-solution ,location)))
        (cram-plan-failures:location-not-reached-failure (f)
          (declare (ignore f))
          (ros-warn (longterm) "Cannot reach location. Retrying.")
          (cpl:retry)))
     (let ((side (var-value
                  '?side
                  (lazy-car (crs:prolog `(cram-plan-library:object-in-hand ,,object ?side))))))
       (prog1
           ,(cond (stationary
                   `(achieve `(cram-plan-library::object-put ,,object ,,location)))
                  (t
                   `(achieve `(cram-plan-library::object-placed-at ,,object ,,location))))
         (ensure-arms-up side)))))
