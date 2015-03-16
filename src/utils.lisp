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

(defmethod init-ms-belief-state (&key debug-window objects)
  (ros-info (bullet) "Clearing bullet world")
  (crs:prolog `(btr:clear-bullet-world))
  (let* ((test-0 (ros-info (bullet) "Gathering data"))
         (robot-pose (get-robot-pose))
         (test-01 (ros-info (bullet) "Robot pose OK"))
         (urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (test-02 (ros-info (bullet) "Robot description OK"))
         (urdf-kitchen
           (cl-urdf:parse-urdf
            (roslisp:get-param "kitchen_description")))
         (test-03 (ros-info (bullet) "Kitchen description OK"))
         (scene-rot-quaternion (tf:euler->quaternion :az pi))
         (scene-rot `(,(tf:x scene-rot-quaternion)
                      ,(tf:y scene-rot-quaternion)
                      ,(tf:z scene-rot-quaternion)
                      ,(tf:w scene-rot-quaternion)))
         (scene-trans `(-3.45 -4.35 0))
         (robot-pose robot-pose)
         (robot-rot `(,(tf:x (tf:orientation robot-pose))
                      ,(tf:y (tf:orientation robot-pose))
                      ,(tf:z (tf:orientation robot-pose))
                      ,(tf:w (tf:orientation robot-pose))))
         (robot-trans `(,(tf:x (tf:origin robot-pose))
                        ,(tf:y (tf:origin robot-pose))
                        ,(tf:z (tf:origin robot-pose))))
         (test-1 (ros-info (bullet) "Asserting scene"))
         (bdgs
           (car
            (force-ll
             (crs:prolog
              `(and (btr:clear-bullet-world)
                    (btr:bullet-world ?w)
                    (btr:assert (btr:object
                                 ?w btr:static-plane floor
                                 ((0 0 0) (0 0 0 1))
                                 :normal (0 0 1) :constant 0))
                    ,@(when debug-window
                       `((btr:debug-window ?w)))
                    (btr:robot ?robot)
                    ,@(loop for object in objects
                            for obj-name = (car object)
                            for obj-pose = (cdr object)
                            collect `(btr:assert
                                      (btr:object
                                       ?w btr:box ,obj-name
                                       ((,(tf:x (tf:origin obj-pose))
                                         ,(tf:y (tf:origin obj-pose))
                                         ,(tf:z (tf:origin obj-pose)))
                                        (,(tf:x (tf:orientation obj-pose))
                                         ,(tf:y (tf:orientation obj-pose))
                                         ,(tf:z (tf:orientation obj-pose))
                                         ,(tf:w (tf:orientation obj-pose))))
                                       :mass 0.0 :size (0.1 0.1 0.1))))
                    (assert (btr:object
                             ?w btr:urdf ?robot
                             (,robot-trans ,robot-rot)
                             :urdf ,urdf-robot))
                    (assert (btr:object
                             ?w btr:semantic-map sem-map-kitchen
                             (,scene-trans ,scene-rot)
                             :urdf ,urdf-kitchen))))))))
    (declare (ignore test-0 test-01 test-02 test-03 test-1))
    (ros-info (bullet) "Check binding for robot")
    (var-value
     '?pr2
     (lazy-car
      (crs:prolog
       `(and (btr:robot ?robot)
             (btr:%object ?w ?robot ?pr2)) bdgs)))
    (ros-info (bullet) "Check binding for kitchen")
    (var-value
     '?sem-map
     (lazy-car
      (crs:prolog
       `(btr:%object ?w sem-map-kitchen ?sem-map) bdgs)))
    (ros-info (bullet) "Done")
    (robosherlock-pm::ignore-bullet-object 'sem-map-kitchen)
    (robosherlock-pm::ignore-bullet-object 'common-lisp::floor)
    (robosherlock-pm::ignore-bullet-object 'cram-pr2-knowledge::pr2)))

(defun prepare-settings ()
  (setf *wait-for-trigger* nil)
  (setf location-costmap::*fixed-frame* "/map")
  ;; NOTE(winkler): This validator breaks IK based `to reach' and `to
  ;; see' location resolution. Disabling it, since everything works
  ;; just nicely without it. Gotta look into this later.
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  ;(cram-designators:disable-location-validation-function
  ; 'spatial-relations-costmap::potential-field-costmap-pose-function)
  ;(cram-designators:disable-location-validation-function
  ; 'spatial-relations-costmap::collision-pose-validator)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20)
  (beliefstate::enable-logging t)
  (ros-info (longterm) "Init Belief State")
  (init-ms-belief-state :debug-window t)
  (setf btr::*bb-comparison-validity-threshold* 0.1)
  (moveit:clear-collision-environment)
  ;; Twice, because sometimes a ROS message for an object gets lost.
  ;(sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

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
