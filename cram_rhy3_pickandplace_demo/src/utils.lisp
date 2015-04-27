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

(in-package :cram-rhy3-pickandplace-demo)

(defstruct (demo-handle (:conc-name dh-))
  (control-command-subscriber nil)
  (control-command nil)
  (control-command-watcher nil)
  (control-command-publisher nil)
  (marker-relative-poses nil)
  (loc-in-front-of-oven nil)
  (loc-in-front-of-island nil)
  (obj-tray nil)
  (obj-marker nil)
  (obj-spoon nil)
  (obj-tomato-sauce nil)
  (name-handle-drawer nil)
  (name-handle-fridge nil))

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
     `(("428" ,(cl-transforms:make-pose (cl-transforms:make-3d-vector 0.01 0.015 0.278)
                                        (cl-transforms:euler->quaternion
                                         :ay (- (/ pi -2) 0.028) :az 0.016)))
       ("213" ,(cl-transforms:make-pose (cl-transforms:make-3d-vector 0.0 0.3 0.0)
                                        (cl-transforms:euler->quaternion :ay (/ pi -2)))))
     :loc-in-front-of-oven
     (desig:make-designator
      'location
      `((desig-props::pose
         ,(cl-tf:pose->pose-stamped
           "/map" 0.0
           (cl-transforms:make-pose
            (cl-transforms:make-3d-vector 0.538 1.9 0.0) ;;2.035
            (cl-transforms:make-quaternion 0.0 0.0 0.0 -1.0))))
        (desig-props:in-front-of desig-props:oven)))
     :loc-in-front-of-island
     (desig:make-designator
      'location
      `((desig-props::pose
         ,(cl-tf:pose->pose-stamped
           "/map" 0.0
           (cl-transforms:make-pose
            (cl-transforms:make-3d-vector -0.323 1.637 0.0)
            (cl-transforms:make-quaternion 0 0 1 0.03))))
        (desig-props:in-front-of desig-props:island)))
     :obj-tray (make-designator
                'object `((type tray)
                          (carry-handles 2)
                          (at ,(make-designator
                                'location
                                `((on Cupboard)
                                  (name "kitchen_island"))))))
     :obj-spoon (make-designator 'object `((type spoon)))
     :obj-tomato-sauce (make-designator
                        'object `((type desig-props:tomato-sauce)
                                  (distance 0.0)))
     :obj-marker (make-designator 'object `((type armarker)))
     :name-handle-drawer "drawer_sinkblock_upper_handle"
     :name-handle-fridge "drawer_fridge_upper_handle")))

(defun destroy-demo-handle (demo-handle)
  (roslisp:unsubscribe (dh-control-command-subscriber demo-handle))
  (roslisp:unadvertise "/demo_command"))

(defun initialize-demo-setup (demo-handle robot &key enable-logging)
  "Initializes the demo setup. `demo-handle' is an initialized
instance of the demo variables, while `robot' is a symbol denoting
either `:pr2' or `:boxy', depending on which top-level plan called this
function. The parameter `enable-logging' either enables logging, or
disables it (default)."
  (initialize-demo-setup-generic demo-handle)
  (ecase robot
    (:pr2 (ros-info (rh demo) "Initializing demo for PR2")
     (initialize-demo-setup-pr2 demo-handle))
    (:boxy (ros-info (rh demo) "Initializing demo for Boxy")
     (initialize-demo-setup-boxy demo-handle)))
  (beliefstate::enable-logging enable-logging))

(defun initialize-demo-setup-generic (demo-handle)
  (declare (ignorable demo-handle))
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20))

(defun initialize-demo-setup-pr2 (demo-handle)
  (declare (ignorable demo-handle))
  ;; Register the location validation function that handles special
  ;; purpose locations for the demo
  (setf location-costmap::*fixed-frame* "/map")
  (cram-designators:register-location-validation-function
   1 robohow-demo-location-validator)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  ;; kick bullet, moveit, and semantic maps
  (initialize-bullet :pr2 :debug-window t)
  (moveit:clear-collision-environment)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun initialize-demo-setup-boxy (demo-handle)
  (declare (ignorable demo-handle)))

(defun pose->trans (pose)
  `(,(cl-transforms:x (cl-transforms:origin pose))
    ,(cl-transforms:y (cl-transforms:origin pose))
    ,(cl-transforms:z (cl-transforms:origin pose))))

(defun quaternion->rot (q)
  `(,(cl-transforms:x q)
    ,(cl-transforms:y q)
    ,(cl-transforms:z q)
    ,(cl-transforms:w q)))

(defun pose->rot (pose)
  (quaternion->rot (cl-transforms:orientation pose)))

(defun get-robot-pose (&optional (frame-id "/base_link"))
  "Gets the current pose of the coordinate frame `frame-id' w.r.t. the
frame `/map'."
  (cl-tf2:ensure-pose-stamped-transformed
    *tf2*
    (cl-tf:pose->pose-stamped
     frame-id 0.0
     (cl-transforms:make-pose
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation)))
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
         (scene-rot (quaternion->rot (cl-transforms:euler->quaternion :az pi)))
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
       (robosherlock-mini-pm:robosherlock-mini-process-module)
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
    (roslisp:publish adv (cl-tf:pose-stamped->msg pose-stamped))))

(defun move-arms-up (&key allowed-collision-objects side ignore-collisions)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (cl-tf:pose->pose-stamped
      "base_link" (roslisp:ros-time)
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector 0.3 0.5 1.3)
       (cl-transforms:euler->quaternion :ax 0)))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (cl-tf:pose->pose-stamped
      "base_link" (roslisp:ros-time)
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
       (cl-transforms:euler->quaternion :ax 0)))
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
                        (cl-transforms:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (cl-transforms:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (cl-transforms:y center-offset))
                           (+ 0.0
                              (cl-transforms:z center-offset)))
                          (cl-transforms:euler->quaternion
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

(define-hook cram-language::on-begin-speech-act
    (sender receiver content in-reply-to))
(define-hook cram-language::on-finish-speech-act
    (id))

(defun send-kqml (demo-handle sender receiver content
                              &optional in-reply-to)
  (let* ((log-id (first (cram-language::on-begin-speech-act
                         sender receiver content in-reply-to)))
         (content (concatenate 'string "'" content "'"))
         (kqml (cond (in-reply-to
                      (make-instance
                       'acl:kqml-performative-tell
                       :sender sender
                       :receiver receiver
                       :content content))
                     (t
                      (make-instance
                       'acl:kqml-performative-tell
                       :sender sender
                       :receiver receiver
                       :content content
                       :in-reply-to in-reply-to)))))
    (send-control-command demo-handle (acl::kqml->string kqml))
    (cram-language::on-finish-speech-act log-id)))

(defun reply-to-kqml (demo-handle kqml content)
  (send-kqml
   demo-handle
   (acl:receiver kqml)
   (acl:sender kqml)
   content
   (acl:content kqml)))

(defun wait-for-kqml (demo-handle)
  (let ((control-command (get-control-command demo-handle)))
    (acl::string->kqml control-command)))

(defun wait-as-receiver (demo-handle receiver)
  (loop for kqml = (wait-for-kqml demo-handle)
        as is-ok = (string= (acl::receiver kqml)
                            receiver)
        when is-ok
          do (return kqml)))

;;;
;;; Plan Macros
;;;

(defmacro with-experiment-active (robot &body body)
  (labels ((combine-symbols (symbols)
             (intern (format nil "~{~a~}" symbols))))
    (assert (or (eql robot :pr2) (eql robot :boxy)))
    `(top-level
       (,(combine-symbols `("WITH-PROCESS-MODULES-"
                            ,(symbol-name robot)))
        (let ((dh (get-demo-handle)))
          (setf *dh* dh)
          (initialize-demo-setup dh ,robot)
          (prog1 (progn ,@body)
            (destroy-demo-handle dh)))))))

(defmacro ensure-results (function)
  `(loop for results = (funcall ,function)
         while (not results)
         finally (return results)))

(defun perceive-a (object &key stationary (move-head t) equate)
  (let ((perceived-object
          (cpl:with-failure-handling
              ((cram-plan-failures:object-not-found (f)
                 (declare (ignore f))
                 (ros-warn
                  (longterm) "Object not found. Retrying.")
                 (cpl:retry)))
            (cond (stationary
                   (let ((at (desig-prop-value
                              object 'desig-props:at)))
                     (when move-head
                       (achieve `(cram-plan-library:looking-at
                                  ,(reference at))))
                     (first (perceive-object
                             'cram-plan-library:currently-visible
                             object))))
                  (t (cpl:with-failure-handling
                         ((cram-plan-failures:location-not-reached-failure (f)
                            (declare (ignore f))
                            (cpl:retry)))
                       (perceive-object 'cram-plan-library:a
                                        object)))))))
    (when equate
      (equate object perceived-object))
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

(defun pick-object (object &key stationary side)
  (when side
    (let ((new-description (append (description object)
                                   `((desig-props::sides (,side)))
                                   (when (eql (desig-prop-value
                                               object 'type)
                                              'tomato-sauce)
                                     `((distance 0.03))))))
      (setf (slot-value object 'cram-designators::description)
            new-description)))
  (cpl:with-retry-counters ((retry-task 0))
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
      (cond (stationary
             (achieve `(cram-plan-library:object-picked ,object)))
            (t
             (achieve `(cram-plan-library:object-in-hand ,object)))))))

(defun place-object (object location &key stationary)
  (cpl:with-failure-handling
      ((cram-plan-failures:manipulation-pose-unreachable (f)
         (declare (ignore f))
         (cram-plan-library::retry-with-updated-location
          location (next-solution location)))
       (cram-plan-failures:location-not-reached-failure (f)
         (declare (ignore f))
         (ros-warn (longterm) "Cannot reach location. Retrying.")
         (cpl:retry)))
    (let ((side (var-value
                 '?side
                 (lazy-car (crs:prolog
                            `(cram-plan-library:object-in-hand
                              ,object ?side))))))
      (prog1
          (cond (stationary
                 (achieve `(cram-plan-library::object-put
                            ,object ,location)))
                (t
                 (achieve `(cram-plan-library::object-placed-at
                            ,object ,location))))
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
                     (cl-tf2:ensure-pose-stamped-transformed
                      *tf2* pose-stamped
                      (cl-tf:frame-id solution))))
               (<= (cl-transforms:v-dist (cl-transforms:make-3d-vector
                                          (cl-transforms:x (cl-transforms:origin pose-stamped-map))
                                          (cl-transforms:y (cl-transforms:origin pose-stamped-map))
                                          0.0)
                                         (cl-transforms:make-3d-vector
                                          (cl-transforms:x (cl-transforms:origin solution))
                                          (cl-transforms:y (cl-transforms:origin solution))
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
               :accept)))
          (t :unknown))))

;; Location utility quick functions

(defmacro in-front-of (location try-indefinitely &body body)
  `(cpl:with-retry-counters ((ctr 3))
     (cpl:with-failure-handling
         ((cram-plan-failures:location-not-reached-failure (f)
            (declare (ignore f))
            (when ,try-indefinitely
              (cpl:do-retry ctr
                (cpl:retry))
              (return t))
            (unless ,try-indefinitely
              (return t))))
       (at-location (,location)
         ,@body))))

(defmacro in-front-of-oven (demo-handle &body body)
  `(in-front-of (dh-loc-in-front-of-oven ,demo-handle) t
     ,@body))

(defmacro in-front-of-island (demo-handle &body body)
  `(in-front-of (dh-loc-in-front-of-island ,demo-handle) t
     ,@body))

(defun look-into-drawer ()
  (let* ((look-at-pose
           (cl-tf:make-pose-stamped
            "map" 0.0
            (tf:make-3d-vector 0.8 0.78 1.0)
            (tf:make-identity-rotation)))
         (look-at-pose-in-base-link
           (cl-tf2:ensure-pose-stamped-transformed
            *tf2*
            look-at-pose
            "base_link" :use-current-ros-time t)))
    (with-designators
        ((look-at (action
                   `((desig-props::type desig-props::trajectory)
                     (desig-props::to desig-props:see)
                     (desig-props::pose ,look-at-pose-in-base-link)))))
      (perform look-at))))

(defun look-front ()
  (with-designators
      ((look-at (action
                 `((desig-props::type desig-props::trajectory)
                   (desig-props::to desig-props:see)
                   (desig-props::pose
                    ,(tf:make-pose-stamped
                      "base_link" 0.0
                      (tf:make-3d-vector 1.0 0.0 0.8)
                      (tf:make-identity-rotation)))))))
    (perform look-at)))

(defun look-front-right ()
  (with-designators
      ((look-at (action
                 `((desig-props::type desig-props::trajectory)
                   (desig-props::to desig-props:see)
                   (desig-props::pose
                    ,(tf:make-pose-stamped
                      "base_link" 0.0
                      (tf:make-3d-vector 1.0 -0.8 0.8)
                      (tf:make-identity-rotation)))))))
    (perform look-at)))

(defun go-in-front-of-drawer ()
  (in-front-of (make-designator
                'location
                `((desig-props:pose
                   ,(tf:make-pose-stamped
                     "map" 0.0
                     (tf:make-3d-vector 0.638 0.788 0.0)
                     (tf:make-quaternion 0 0 0 1)))))
      t))

(defun go-in-front-of-island-2 ()
  (in-front-of
      (desig:make-designator
       'location
       `((desig-props::pose
          ,(cl-tf:pose->pose-stamped
            "/map" 0.0
            (cl-transforms:make-pose
             (cl-transforms:make-3d-vector -0.323 1.137 0.0)
             (cl-transforms:make-quaternion 0 0 1 0.03))))
         (desig-props:in-front-of desig-props:island)))
      t))

(defun go-in-front-of-fridge ()
  (in-front-of (make-designator
                'location
                `((desig-props:pose
                   ,(tf:make-pose-stamped
                     "map" 0.0
                     (tf:make-3d-vector 0.4874 -0.4438 0.05)
                     (tf:make-quaternion -0.01 -0.003 -0.126 0.992)))))
      t))

(defun go-in-front-of-fridge-2 ()
  (in-front-of (make-designator
                'location
                `((desig-props:pose
                   ,(tf:make-pose-stamped
                     "map" 0.0;;0.483
                     (tf:make-3d-vector 0.383 -0.861 0.05)
                     (tf:make-quaternion 0.0 -0.002 0.005 1.0)))))
      nil))

(defun go-in-front-of-fridge-3 ()
  (in-front-of (make-designator
                'location
                `((desig-props:pose
                   ,(tf:make-pose-stamped
                     "map" 0.0;0.747
                     (tf:make-3d-vector 1.0 -0.51 0.05)
                     (tf:euler->quaternion)))))
      nil))

(defun go-in-front-of-fridge-4 ()
  (in-front-of (make-designator
                'location
                `((desig-props:pose
                   ,(tf:make-pose-stamped
                     "map" 0.0
                     (tf:make-3d-vector 0.416 -0.881 0.0)
                     (tf:euler->quaternion)))))
      nil))

;;;
;;; Spoon
;;;

(defun perceive-spoon (demo-handle)
  (let* ((orig (dh-obj-spoon demo-handle))
         (object (ensure-results
                   (perceive-a orig
                               :stationary t
                               :move-head nil)))
         (pose (reference (desig-prop-value
                           object 'at)))
         (transformed
           (tf:copy-pose-stamped
            (tf:pose->pose-stamped
             (tf:frame-id pose) 0.0
             (cl-transforms:transform-pose
              (cl-transforms:make-transform
               (tf:make-identity-vector)
               (tf:euler->quaternion :ax pi :az (/ pi 2)))
              (tf:make-pose
               (tf:make-identity-vector)
               (tf:orientation pose))))
            :origin (tf:origin pose)))
         (desig (make-designator
                 'object
                 (append (remove-if
                          (lambda (a)
                            (equal (car a) 'at))
                          (description object))
                         `((at
                            ,(make-designator
                              'location
                              `((pose
                                 ,transformed))))))))
         (data (slot-value object 'desig:data)))
    (setf (slot-value data 'pose) transformed)
    (setf (slot-value desig 'desig:data) data)
    (setf (slot-value orig 'desig:successor) desig)
    (setf (slot-value desig 'desig:parent) orig)
    ;desig
    object))

;;;
;;; Ketchup
;;;

(defun perceive-tomato-sauce (demo-handle)
  (perceive-a (dh-obj-tomato-sauce demo-handle)
              :stationary t
              :move-head nil))

;;;
;;; Tray
;;;

(defun perceive-tray (demo-handle)
  (let ((trays (perceive-all (dh-obj-tray demo-handle)
                             :stationary t
                             :move-head nil)))
    (labels ((area-of-object (object)
               (let* ((segment (desig-prop-value object 'segment))
                      (dimensions-2d (cadr (assoc 'dimensions-2d segment))))
                 (* (elt dimensions-2d 0) (elt dimensions-2d 1)))))
      (let ((tray
              (find (loop for tray in trays
                          maximizing (area-of-object tray))
                    trays :test (lambda (area object)
                                  (equal area (area-of-object object))))))
        (when tray
          (equate (dh-obj-tray demo-handle) tray))))))

;;;
;;; Drawer and Fridge
;;;

(defun perceive-handle-drawer (demo-handle)
  (let ((handles (perceive-all (make-designator
                                'object
                                `((desig-props:type
                                   desig-props::semantic-handle)))
                               :stationary t
                               :move-head nil)))
    (find (dh-name-handle-drawer demo-handle)
          handles
          :test
          (lambda (name handle)
            (string= (desig-prop-value handle 'desig-props:name)
                     name)))))

(defun perceive-handle-fridge (demo-handle)
  (let ((handles (perceive-all (make-designator
                                'object
                                `((desig-props:type
                                   desig-props::semantic-handle)))
                               :stationary t
                               :move-head nil)))
    (find (dh-name-handle-fridge demo-handle)
          handles
          :test
          (lambda (name handle)
            (string= (desig-prop-value handle 'desig-props:name)
                     name)))))

;;;
;;; Markers
;;;

(defun look-at-marker-suitable-pose ()
  (achieve `(cram-plan-library:looking-at
             ,(cl-tf:pose->pose-stamped
               "/base_link" 0.0
               (cl-transforms:make-pose
                (cl-transforms:make-3d-vector 1.0 0.0 1.0)
                (cl-transforms:euler->quaternion))))))

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
                    (cl-tf:pose->pose-stamped
                     "/map" 0.0
                     (cl-transforms:transform-pose
                      (cl-transforms:pose->transform marker-at)
                      (cl-transforms:transform-pose
                       (cl-transforms:transform-inv
                        (cl-transforms:pose->transform marker-relative-pose))
                       (cl-transforms:transform->pose
                        (cl-transforms:transform-inv (cl-transforms:pose->transform
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

(defun human-tracking (demo-handle)
  (send-kqml demo-handle "Boxy" "*" "I am waiting for a human in the scene now.")
  (wait-for-human-in-scene demo-handle)
  (send-kqml demo-handle "Boxy" "*" "A human entered the scene.")
  (wait-while-human-in-scene demo-handle)
  (send-kqml demo-handle "Boxy" "*" "The human left the scene again.")
  (send-kqml demo-handle "Boxy" "PR2" "Come back to the table.")
  (reply-to-kqml
   demo-handle
   (wait-as-receiver demo-handle "Boxy")
   "Welcome back."))

(defun wait-for-human-in-scene (demo-handle)
  (perceive-tracked-human demo-handle))
 
(defun wait-while-human-in-scene (demo-handle)
  (loop while (is-human-in-scene demo-handle)))

(defun is-message-on-topic (demo-handle topic topic-type)
  (declare (ignore demo-handle))
  (let ((message-present nil))
    (labels ((message-function (msg)
               (declare (ignore msg))
               (setf message-present t)))
      (let ((subscriber (roslisp:subscribe
                         topic topic-type
                         #'message-function)))
        (sleep 1.0)
        (roslisp:unsubscribe subscriber)
        message-present))))

(defun is-human-in-scene (demo-handle)
  (is-message-on-topic
   demo-handle
   "/RoboSherlock_jworch/person"
   "person_msgs/Person"))

(defun perceive-tracked-human (demo-handle)
  (loop until (is-human-in-scene demo-handle)))

(defun relative-linear-arm-translation->trajectory
    (arm rel-position &key (ignore-collisions t)
                        (raise-elbow t))
  (let* ((id-pose
           (tf:pose->pose-stamped
            (case arm
              (:left "l_wrist_roll_link")
              (:right "r_wrist_roll_link"))
            0.0 (tf:make-identity-pose)))
         (tl-pose
           (cl-tf2:ensure-pose-stamped-transformed
            *tf2* id-pose "torso_lift_link"
            :use-current-ros-time t))
         (tl-translated-pose
           (tf:copy-pose-stamped
            tl-pose
            :origin (tf:v+ (tf:origin tl-pose)
                           rel-position))))
    (pr2-manip-pm::arm-pose->trajectory
     arm tl-translated-pose
     :ignore-collisions ignore-collisions
     :raise-elbow (when raise-elbow arm))))

(defmacro ensure-results (&body body)
  `(loop as result = (cpl:with-failure-handling
                         ((cram-plan-library::object-not-found (f)
                            (declare (ignore f))
                            (cpl:retry)))
                       ,@body)
         while (not result)
         finally (return result)))

(defmacro ensure-manipulation (&body body)
  `(cpl:with-failure-handling
       ((cram-plan-library::manipulation-failure (f)
          (declare (ignore f))
          (cpl:retry)))
     ,@body))

(defun get-tomato-sauce (demo-handle)
  (ensure-arms-up)
  (go-in-front-of-fridge)
  (look-front)
  (let ((pose-map nil))
    (labels ((move-handle-relative-pose
                 (side vector ignore-collisions
                  &optional (orientation (cl-transforms:euler->quaternion)))
               (pr2-manip-pm::execute-move-arm-pose
                side
                (cl-tf:copy-pose-stamped
                 pose-map
                 :origin (cl-transforms:v+
                          (cl-transforms:origin pose-map)
                          vector)
                 :orientation orientation)
                :ignore-collisions ignore-collisions))
             (move-arm-relative-pose (side vector ignore-collisions)
               (let ((traj
                       (relative-linear-arm-translation->trajectory
                        side vector
                        :ignore-collisions ignore-collisions
                        :raise-elbow nil)))
                 (moveit:execute-trajectory traj)))
             (move-base-relative-pose (vector)
               (let* ((base-id
                        (cl-tf2:ensure-pose-stamped-transformed
                         *tf2*
                         (cl-tf:pose->pose-stamped
                          "base_footprint" 0.0
                          (cl-transforms:make-identity-pose))
                         "map"))
                      (base-translated
                        (cl-tf:copy-pose-stamped
                         base-id
                         :origin (cl-tf:v+ (cl-tf:origin base-id)
                                           vector))))
                 (let ((action (make-designator
                                'action `((desig-props:type
                                           desig-props:navigation)
                                          (desig-props:goal
                                           ,(make-designator
                                             'location
                                             `((desig-props:pose
                                                ,base-translated))))))))
                   (perform action)))))
      (cpl:with-failure-handling
          ((cram-plan-failures::manipulation-failed (f)
             (declare (ignore f))
             (cpl:retry)))
        (let* ((hndl (ensure-results (perceive-handle-fridge demo-handle)))
               (pose (desig-prop-value hndl 'pose))
               (pose-map-new
                 (cl-tf2:ensure-pose-stamped-transformed
                  *tf2* pose "map" :use-current-ros-time t)))
          (setf pose-map pose-map-new)
          (when hndl
            (ensure-manipulation
              (move-handle-relative-pose
               :right (cl-transforms:make-3d-vector -0.3 0.03 0.0) nil))
            (pr2-manip-pm::open-gripper :right)
            (ensure-manipulation
              (move-handle-relative-pose
               :right (cl-transforms:make-3d-vector -0.19 0.03 0.0) t))
            (pr2-manip-pm::close-gripper :right)
            (when (< (pr2-manip-pm::get-gripper-state :right) 0.0025)
              (pr2-manip-pm::open-gripper :right)
              (cpl:fail 'cram-plan-failures::manipulation-failed))
            (ensure-manipulation
              (move-handle-relative-pose
               :right (cl-transforms:make-3d-vector -0.4 -0.15 0.0) t
               (cl-transforms:euler->quaternion :az 0.7)))
            (ensure-manipulation
              (move-handle-relative-pose
               :right (cl-transforms:make-3d-vector -0.45 -0.30 0.0) t
               (cl-transforms:euler->quaternion :az 1.0)))
            (ensure-manipulation
              (move-handle-relative-pose
               :right (cl-transforms:make-3d-vector -0.45 -0.40 0.0) t
               (cl-transforms:euler->quaternion :az 1.2)))
            (pr2-manip-pm::open-gripper :right)
            (ensure-manipulation
              (move-arm-relative-pose
               :right (cl-transforms:make-3d-vector -0.1 -0.10 0.0) t))
            (ensure-manipulation
              (pr2-manip-pm::execute-move-arm-pose
               :right (tf:make-pose-stamped
                       "base_link" 0.0
                       (tf:make-3d-vector -0.2 -0.5 1.4)
                       (tf:euler->quaternion
                        :ay (/ pi -2)))))
            (go-in-front-of-fridge-2)
            (pr2-manip-pm::close-gripper :left)
            (ensure-manipulation
              (pr2-manip-pm::execute-move-arm-pose
               :left (tf:make-pose-stamped
                      "base_link" 0.0
                      (tf:make-3d-vector 0.55 0.3 0.95)
                      (tf:euler->quaternion :az (/ pi -2)))
               :ignore-collisions t))
            (ensure-manipulation
              (pr2-manip-pm::execute-move-arm-pose
               :left (tf:make-pose-stamped
                      "base_link" 0.0
                      (tf:make-3d-vector 0.55 -0.2 0.95)
                      (tf:euler->quaternion :az (/ pi -2)))
               :ignore-collisions t))
            (ensure-arms-up :left)
            (go-in-front-of-fridge-3)
            (look-front-right)
            (let ((tomato-sauce (ensure-results (perceive-tomato-sauce demo-handle))))
              (setf pr2-manip-pm::*raise-elbow* nil)
              (moveit::without-collision-objects
                  `("HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#DRAWER_FRIDGE_UPPER-0"
                    "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#DRAWER_FRIDGE_UPPER-1"
                    "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#DRAWER_FRIDGE_UPPER-2"
                    "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#DRAWER_FRIDGE_UPPER-3"
                    "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#DRAWER_FRIDGE_UPPER-4" ,(string (desig-prop-value tomato-sauce 'desig-props::name)))
                (ensure-manipulation
                  (cpl:with-failure-handling
                      ((cram-plan-failures:object-not-found (f)
                         (declare (ignore f))
                         (cpl:retry)))
                    (pr2-manip-pm::execute-move-arm-pose
                     :right
                     (tf:make-pose-stamped
                      "base_link" 0.0
                      (tf:make-3d-vector -0.1 -0.5 1.4)
                      (tf:euler->quaternion :az (/ pi 2)))
                     :ignore-collisions t)
                    (pick-object
                     tomato-sauce :stationary t :side :left))))
              (setf pr2-manip-pm::*raise-elbow* t)
              (pr2-manip-pm::execute-move-arm-pose
               :right
               (tf:make-pose-stamped
                "base_link" 0.0
                (tf:make-3d-vector -0.1 -0.5 1.4)
                (tf:euler->quaternion :az (/ pi 2)))
               :ignore-collisions t)
              (go-in-front-of-fridge-4)
              (pr2-manip-pm::execute-move-arm-pose
               :right
               (tf:make-pose-stamped
                "base_link" 0.0
                (tf:make-3d-vector -0.1 -0.8 1.4)
                (tf:euler->quaternion)))
              (pr2-manip-pm::execute-move-arm-pose
               :right
               (tf:make-pose-stamped
                "base_link" 0.0
                (tf:make-3d-vector 0.1 -0.8 1.2)
                (tf:euler->quaternion)))
              (pr2-manip-pm::execute-move-arm-pose
               :right
               (tf:make-pose-stamped
                "base_link" 0.0
                (tf:make-3d-vector 0.3 0.0 1.2)
                (tf:euler->quaternion)))
              tomato-sauce)))))))

(defun put-tomato-sauce-on-table (demo-handle tomato-sauce)
  (let* ((target-pose
           (tf:make-pose-stamped
            "map" 0.0
            (tf:make-3d-vector -0.8 1.0 1.05)
            (tf:euler->quaternion :az pi)))
         (loc (make-designator
               'location `((desig-props:pose
                            ,target-pose)))))
    (publish-pose target-pose)
    (ensure-manipulation
      (place-object tomato-sauce loc))))

(defun get-spoon (demo-handle)
  (go-in-front-of-drawer)
  (ensure-arms-up)
  (look-into-drawer)
  (let ((handle (ensure-results (perceive-handle-drawer demo-handle))))
    (when handle
      (let* ((pose (desig-prop-value handle 'pose))
             (pose-map
               (cl-tf2:ensure-pose-stamped-transformed
                *tf2* pose "map" :use-current-ros-time t)))
        (labels ((move-handle-relative-pose
                     (side vector ignore-collisions)
                   (pr2-manip-pm::execute-move-arm-pose
                    side
                    (cl-tf:copy-pose-stamped
                     pose-map
                     :origin (cl-transforms:v+
                              (cl-transforms:origin pose-map)
                              vector)
                     :orientation (cl-transforms:euler->quaternion
                                   :ax (/ pi 2)))
                    :ignore-collisions ignore-collisions))
                 (move-arm-relative-pose (side vector ignore-collisions)
                   (let ((traj
                           (relative-linear-arm-translation->trajectory
                            side vector
                            :ignore-collisions ignore-collisions
                            :raise-elbow nil)))
                     (moveit:execute-trajectory traj)))
                 (move-base-relative-pose (vector)
                   (let* ((base-id
                            (cl-tf2:ensure-pose-stamped-transformed
                             *tf2*
                             (cl-tf:pose->pose-stamped
                              "base_footprint" 0.0
                              (cl-transforms:make-identity-pose))
                             "map"))
                          (base-translated
                            (cl-tf:copy-pose-stamped
                             base-id
                             :origin (cl-tf:v+ (cl-tf:origin base-id)
                                               vector))))
                     (let ((action (make-designator
                                    'action `((desig-props:type
                                               desig-props:navigation)
                                              (desig-props:goal
                                               ,(make-designator
                                                 'location
                                                 `((desig-props:pose
                                                    ,base-translated))))))))
                       (perform action)))))
          (ensure-manipulation
            (move-handle-relative-pose
             :right (cl-transforms:make-3d-vector -0.4 -0.2 0.02) nil))
          (pr2-manip-pm::open-gripper :right)
          (ensure-manipulation
            (move-arm-relative-pose
             :right (cl-transforms:make-3d-vector 0.2 0.0 0.0) t))
          (pr2-manip-pm::close-gripper :right)
          (ensure-manipulation
            (move-arm-relative-pose
             :right (cl-transforms:make-3d-vector -0.2 0.0 0.0) t))
          (move-base-relative-pose (cl-transforms:make-3d-vector
                                    -0.2 0.0 0.0))
          ;; Grasp object here.
          (look-into-drawer)
          (let ((spoon (ensure-results (perceive-spoon demo-handle))))
            (ensure-manipulation
              (pick-object spoon :stationary t :side :left))
            (ensure-manipulation
              (move-arm-relative-pose
               :right (cl-transforms:make-3d-vector 0.4 0.0 0.0) t))
            (pr2-manip-pm::open-gripper :right)
            (ensure-manipulation
              (move-arm-relative-pose
               :right (cl-transforms:make-3d-vector -0.2 0.0 0.0) t))
            (ensure-arms-up)
            spoon))))))

(defun put-spoon-on-table (demo-handle spoon)
  (declare (ignorable demo-handle))
  (let ((spoon-putdown-pose
          (tf:make-pose-stamped
           "map" 0.0
           (tf:make-3d-vector -1.05 1.0 0.83)
           (tf:euler->quaternion
            :ax pi :az pi))))
    (go-in-front-of-island-2)
    (place-object
     spoon
     (make-designator
      'location
      `((desig-props::pose
         ,spoon-putdown-pose)))
     :stationary t)))
    ;; (in-front-of-island demo-handle
    ;;   (ensure-manipulation
    ;;     (place-object
    ;;      spoon
    ;;      (make-designator
    ;;       'location
    ;;       `((desig-props::pose ,spoon-putdown-pose)))
    ;;       ;; `((desig-props:on Cupboard)
    ;;       ;;   (desig-props:name "kitchen_island")))
    ;;      :stationary t)))))

(defun test-spoon-putdown-pose ()
  (let ((spoon-putdown-pose
          (tf:make-pose-stamped
           "map" 0.0
           (tf:make-3d-vector -1 1.2 1)
           (tf:euler->quaternion
            :az pi :ax pi))))
    (roslisp:publish
     (roslisp:advertise "/testpose" "geometry_msgs/PoseStamped")
     (tf:pose-stamped->msg spoon-putdown-pose))))

(defun current-robot-pose ()
  (cl-tf2:ensure-pose-stamped-transformed
   *tf2* (tf:pose->pose-stamped "base_footprint" 0.0
                                (tf:make-identity-pose))
   "map"))

(defun select-rs-instance (name)
  (uima::set-uima-host
   (concatenate 'string "/RoboSherlock_" name))
  (uima::init-uima-bridge))

(defun start-node ()
  (roslisp-utilities:startup-ros)
  (select-rs-instance "handles"))
