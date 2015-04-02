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

(defstruct (demo-handle (:conc-name dh-))
  (control-command-subscriber nil)
  (control-command nil)
  (control-command-watcher nil)
  (control-command-publisher nil)
  (marker-relative-poses nil)
  (loc-in-front-of-oven nil)
  (loc-in-front-of-island nil)
  (obj-tray nil)
  (obj-marker nil))

;;;
;;; Helper functions
;;;

(defun get-demo-handle ()
  (let ((cc-fluent (cpl:make-fluent :name "control-command"
                                    :allow-tracing nil)))
    (make-demo-handle
     :control-command-subscriber
     (roslisp:subscribe "/demo_command" "std_msgs/String"
                        (lambda (msg)
                          (with-fields (data) msg
                            (setf (cpl:value cc-fluent) data))))
     :control-command cc-fluent
     :control-command-watcher (cpl:fl-value-changed
                               cc-fluent :test #'string=)
     :control-command-publisher (roslisp:advertise "/demo_command"
                                                   "std_msgs/String"
                                                   :latch t)
     :marker-relative-poses
     `(("428" ,(tf:make-pose (tf:make-3d-vector 0.01 0.015 0.278)
                             (tf:euler->quaternion
                              :ay (- (/ pi -2) 0.028) :az 0.016)))
       ("213" ,(tf:make-pose (tf:make-3d-vector 0.0 0.3 0.0)
                             (tf:euler->quaternion :ay (/ pi -2)))))
     :loc-in-front-of-oven
     (desig:make-designator
      'location
      `((desig-props::pose
         ,(tf:make-pose-stamped
           "/map" 0.0
           (tf:make-3d-vector 0.538 1.9 0.0) ;;2.035
           (tf:make-quaternion 0.0 0.0 0.0 -1.0)))
        (desig-props:in-front-of desig-props:oven)))
     :loc-in-front-of-island
     (desig:make-designator
      'location
      `((desig-props::pose
         ,(tf:make-pose-stamped
           "/map" 0.0
           (tf:make-3d-vector -0.323 1.437 0.0)
           (tf:make-quaternion 0 0 1 0.03)))
        (desig-props:in-front-of desig-props:island)))
     :obj-tray (make-designator
                'object `((type tray)
                          (at ,(make-designator
                                'location
                                `((on Cupboard)
                                  (name "kitchen_island"))))))
     :obj-marker (make-designator 'object `((type armarker))))))

(defun destroy-demo-handle (demo-handle)
  (roslisp:unsubscribe (dh-control-command-subscriber demo-handle))
  (roslisp:unadvertise "/demo_command"))

(defun initialize-demo-setup (demo-handle robot
                              &key enable-logging)
  "Initializes the demo setup. `demo-handle' is an initialized
instance of the demo variables, while `robot' is a symbol denoting
either `PR2' or `Boxy', depending on which top-level plan called this
function. The parameter `enable-logging' either enables logging, or
disables it (default)."
  (declare (ignorable demo-handle robot))
  ;; Register the location validation function that handles special
  ;; purpose locations for the demo
  (setf location-costmap::*fixed-frame* "/map")
  (cram-designators:register-location-validation-function
   1 robohow-demo-location-validator)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20)
  (initialize-bullet robot :debug-window t)
  (moveit:clear-collision-environment)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (beliefstate::enable-logging enable-logging))

(defun pose->trans (pose)
  `(,(tf:x (tf:origin pose))
    ,(tf:y (tf:origin pose))
    ,(tf:z (tf:origin pose))))

(defun quaternion->rot (q)
  `(,(tf:x q) ,(tf:y q) ,(tf:z q) ,(tf:w q)))

(defun pose->rot (pose)
  (quaternion->rot (tf:orientation pose)))

(defun get-robot-pose (&optional (frame-id "/base_link"))
  "Gets the current pose of the coordinate frame `frame-id' w.r.t. the
frame `/map'."
  (cl-tf2:do-transform
    *tf2*
    (cl-transforms-plugin:make-pose-stamped
     (cl-transforms:make-pose
      (tf:make-identity-vector)
      (tf:make-identity-rotation))
     frame-id
     0.0)
    "/map"))

(defmethod initialize-bullet (robot &key debug-window)
  (crs:prolog `(btr:clear-bullet-world))
  (let* ((robot-pose (get-robot-pose))
         (urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (urdf-kitchen
           (cl-urdf:parse-urdf
            (roslisp:get-param "kitchen_description")))
         (scene-rot (quaternion->rot (tf:euler->quaternion :az pi)))
         (scene-trans `(-3.45 -4.35 0))
         (robot-rot (pose->rot robot-pose))
         (robot-trans (pose->trans robot-pose)))
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
            (assert (btr:object
                     ?w btr:urdf ?robot
                     (,robot-trans ,robot-rot)
                     :urdf ,urdf-robot))
            (assert (btr:object
                     ?w btr:semantic-map sem-map-kitchen
                     (,scene-trans ,scene-rot)
                     :urdf ,urdf-kitchen))))))
  (robosherlock-pm::ignore-bullet-object 'sem-map-kitchen)
  (robosherlock-pm::ignore-bullet-object 'common-lisp::floor)
  (robosherlock-pm::ignore-bullet-object 'cram-pr2-knowledge::pr2))

(defmacro with-process-modules-pr2 (&body body)
  "Register and start all process modules necessary for operating the
PR2."
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defmacro with-process-modules-boxy (&body body)
  "Register and start all process modules necessary for operatinxg
Boxy."
  `(cpm:with-process-modules-running
       ()
     ,@body))

(defmacro try-n-times (n &body body)
  "Try a piece of code `body', recovering from failures `n' times
before escalating the last occured failure to the next higher plan
entity."
  `(cpl:with-retry-counters ((retry-count ,n))
     (cpl:with-failure-handling
         (((or cram-plan-failures:object-not-found
               cram-plan-failures:manipulation-failure
               cram-plan-failures:location-not-reached-failure) (f)
            (declare (ignore f))
            (cpl:do-retry retry-count
              (cpl:retry))))
       ,@body)))

(defmacro try-forever (&body body)
  `(cpl:with-failure-handling
       (((or cram-plan-failures:object-not-found
             cram-plan-failures:manipulation-failure
             cram-plan-failures:location-not-reached-failure) (f)
          (declare (ignore f))
          (cpl:retry)))
     ,@body))

(defun publish-pose (pose-stamped &optional (topic "/object"))
  "Publish the stamped pose `pose-stamped' onto topic `topic'. `topic'
defaults to the topic `/object'."
  (let ((adv (roslisp:advertise topic "geometry_msgs/PoseStamped")))
    (roslisp:publish adv (tf:pose-stamped->msg pose-stamped))))

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
      (tf:euler->quaternion :ax 0))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects)))

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
        collect handle-object))

(defun get-control-command (demo-handle)
  (cpl:wait-for (cpl-impl:fl-pulsed
                 (dh-control-command-watcher demo-handle)))
  (prog1 (cpl:value (dh-control-command demo-handle))
    (setf
     (dh-control-command-watcher demo-handle)
     (cpl:fl-value-changed (dh-control-command demo-handle)
                           :test #'string=))))

(defun wait-for-control-command (demo-handle command)
  (loop while (not (string= (get-control-command demo-handle)
                            command))))

(defun wait-for-control-continue (demo-handle)
  (wait-for-control-command demo-handle "continue"))

(defun send-control-command (demo-handle command)
  (roslisp:publish (dh-control-command-publisher demo-handle)
                   (make-message "std_msgs/String" :data command)))

(defun send-kqml (demo-handle sender receiver content
                              &optional in-reply-to)
  (let ((command
          (cond (in-reply-to
                 (concatenate
                  'string
                  "reply :sender " sender " :receiver " receiver
                  ":content " content ":in-reply-to " in-reply-to))
                (t
                 (concatenate
                  'string
                  "tell :sender " sender " :receiver " receiver
                  ":content " content)))))
    (send-control-command demo-handle command)))

(defun wait-for-kqml (demo-handle)
  (let ((control-command (get-control-command demo-handle)))
    ;; TODO(winkler): Split the string `control-command' here, check
    ;; for primary expression (first word) and then split the
    ;; `keyword' constructs after that (pairs of `:keyword
    ;; value'). Then, return it as a hash table.
    (make-hash-table)))

(defun wait-as-receiver (demo-handle receiver)
  (loop for kqml = (wait-for-kqml demo-handle)
        as is-ok = (not (and (string=
                              (gethash "receiver" kqml)
                              receiver)))
        when is-ok
          do (return kqml)))

;;;
;;; Plan Macros
;;;

(defmacro ensure-results (function)
  `(loop for results = (funcall ,function)
         while (not results)
         finally (return results)))

(defmacro perceive-a (object &key stationary (move-head t) equate)
  `(let ((perceived-object
           (cpl:with-failure-handling
               ((cram-plan-failures:object-not-found (f)
                  (declare (ignore f))
                  (ros-warn
                   (longterm) "Object not found. Retrying.")
                  (cpl:retry)))
             (cond (,stationary
                    (let ((at (desig-prop-value
                               ,object 'desig-props:at)))
                      (when ,move-head
                        (achieve `(cram-plan-library:looking-at
                                   ,(reference at))))
                      (first (perceive-object
                              'cram-plan-library:currently-visible
                              ,object))))
                   (t (cpl:with-failure-handling
                          ((cram-plan-failures:location-not-reached-failure (f)
                             (declare (ignore f))
                             (cpl:retry)))
                        (perceive-object 'cram-plan-library:a
                                         ,object)))))))
     (when ,equate
       (equate ,object perceived-object))
     perceived-object))

(defmacro perceive-all (object &key stationary (move-head t))
  `(cpl:with-failure-handling
       ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (ros-warn (longterm) "Object not found. Retrying.")
          (cpl:retry)))
     (cond (,stationary
            (let ((at (desig-prop-value ,object 'desig-props:at)))
              (when ,move-head
                (achieve `(cram-plan-library:looking-at ,(reference at))))
              (perceive-object
               'cram-plan-library:currently-visible
               ,object)))
           (t (cpl:with-failure-handling
                  ((cram-plan-failures:location-not-reached-failure (f)
                     (declare (ignore f))
                     (cpl:retry)))
                (perceive-object 'cram-plan-library:all ,object))))))

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

;;;
;;; Locations
;;;

;;; Validators

(defun robohow-demo-location-validator (designator solution)
  "This validator is used for validating all semantic locations used
throughout the demo experiment."
  (labels ((pose-within-distance (pose-stamped
                                  &optional (threshold 0.10))
             (let ((pose-stamped-map
                     (cl-tf2:do-transform
                      *tf2* pose-stamped
                       (cl-tf2:get-frame-id solution))))
               (<= (tf:v-dist (tf:make-3d-vector
                               (tf:x (tf:origin pose-stamped-map))
                               (tf:y (tf:origin pose-stamped-map))
                               0.0)
                              (tf:make-3d-vector
                               (tf:x (tf:origin solution))
                               (tf:y (tf:origin solution))
                               0.0))
                   threshold))))
    (cond ((or (eql (desig-prop-value
                     designator 'desig-props:in-front-of)
                    'desig-props:oven)
               (eql (desig-prop-value
                     designator 'desig-props:in-front-of)
                    'desig-props:island))
           (let ((pose (desig-prop-value designator 'desig-props:pose)))
             (when (and pose (pose-within-distance pose))
               :accept))))))

;; Location utility quick functions

(defmacro in-front-of (location try-indefinitely &body body)
  `(cpl:with-failure-handling
       ((cram-plan-failures:location-not-reached-failure (f)
          (declare (ignore f))
          (when ,try-indefinitely
            (cpl:retry))))
     (at-location (,location)
       ,@body)))

(defmacro in-front-of-oven (demo-handle &body body)
  `(in-front-of (dh-loc-in-front-of-oven ,demo-handle) t
     ,@body))

(defmacro in-front-of-island (demo-handle &body body)
  `(in-front-of (dh-loc-in-front-of-island ,demo-handle) t
     ,@body))

;;;
;;; Tray
;;;

(defun perceive-tray (demo-handle)
  (perceive-a (dh-obj-tray demo-handle)
              :stationary t
              :move-head nil
              :equate t))

;;;
;;; Markers
;;;

(defun look-at-marker-suitable-pose ()
  (achieve `(cram-plan-library:looking-at
             ,(tf:make-pose-stamped
               "/base_link" 0.0
               (tf:make-3d-vector 1.0 0.0 1.0)
               (tf:euler->quaternion)))))

(defun perceive-markers (demo-handle)
  (perceive-all (dh-obj-marker demo-handle)
                :stationary t
                :move-head nil))

(defun marker-id->pose (marker-pose-pairs id)
  (cadr (find id marker-pose-pairs
              :test (lambda (id marker-pose-pair)
                      (destructuring-bind (marker pose)
                          marker-pose-pair
                        (declare (ignore pose))
                        (string= marker id))))))

(defun markers-relative-pose->absolute-poses
    (demo-handle perceived-markers relative-pose)
  (mapcar (lambda (perceived-marker)
            (when (eql (desig-prop-value perceived-marker 'type)
                       'armarker)
              (let ((marker-id (desig-prop-value perceived-marker 'id)))
                (when marker-id
                  (let ((marker-relative-pose (marker-id->pose
                                               (dh-marker-relative-poses
                                                demo-handle)
                                               marker-id))
                        (marker-at (reference (desig-prop-value
                                               perceived-marker 'at))))
                    (tf:pose->pose-stamped
                     "/map" 0.0
                     (cl-transforms:transform-pose
                      (tf:pose->transform marker-at)
                      (cl-transforms:transform-pose
                       (tf:transform-inv
                        (tf:pose->transform marker-relative-pose))
                       (tf:transform->pose
                        (tf:transform-inv (tf:pose->transform
                                           relative-pose)))))))))))
          perceived-markers))

(defmacro with-logging-enabled (&body body)
  `(let ((logging-state (beliefstate:logging-enabled)))
     (beliefstate:enable-logging t)
     (progn ,@body)
     (beliefstate:enable-logging logging-state)))

(defmacro with-logging-disabled (&body body)
  `(let ((logging-state (beliefstate:logging-enabled)))
     (beliefstate:enable-logging nil)
     (progn ,@body)
     (beliefstate:enable-logging logging-state)))

(defun wait-for-kqml-message (demo-handle sender receiver content)
  (declare (ignore demo-handle sender receiver content))
  (ros-error (demo) "IMPLEMENT ME: `wait-for-kqml-message'"))

(defun wait-for-human-near-oven (demo-handle)
  (declare (ignore demo-handle))
  (ros-error (demo) "IMPLEMENT ME: `wait-for-human-near-oven'"))

(defun is-message-on-topic (demo-handle topic topic-type)
  (declare (ignore demo-handle))
  (let ((message-present nil))
    (labels ((message-function (msg)
               (declare (ignore msg))
               (setf message-present t)))
      (let ((subscriber (roslisp:subscribe
                         topic topic-type
                         #'message-function)))
        (sleep 0.5)
        (roslisp:unsubscribe subscriber)
        message-present))))

(defun perceive-tracked-human (demo-handle)
  (loop until (is-message-on-topic
               demo-handle
               "/RoboSherlock_jworch/person"
               "person_msgs/Person")))
