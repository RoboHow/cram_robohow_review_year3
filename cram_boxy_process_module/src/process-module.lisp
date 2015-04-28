;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name or Universitaet Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :boxy-pm)

(defparameter *boxy-pm-handle* nil)

(defstruct (boxy-pm-handle (:conc-name boxy-))
  (controller-manager nil)
  (right-arm nil)
  (left-arm nil)
  (left-arm-vel-mux nil)
  (right-arm-vel-mux nil))
  
(defun init-boxy-pm-handle ()
  (let ((controller-manager
          (make-instance
           'persistent-service
           :service-name "/controller_manager/switch_controller"
           :service-type "controller_manager_msgs/SwitchController"))
        (right-arm-joint-controller
          (actionlib-lisp:make-simple-action-client
           "/r_arm_traj_controller/follow_joint_trajectory"
           "control_msgs/FollowJointTrajectoryAction"))
        (left-arm-joint-controller
          (actionlib-lisp:make-simple-action-client
           "/r_arm_traj_controller/follow_joint_trajectory"
           "control_msgs/FollowJointTrajectoryAction"))
        (left-arm-vel-mux (init-mux-handle "/l_arm_vel/mux"))
        (right-arm-vel-mux (init-mux-handle "/r_arm_vel/mux"))

)
    (make-boxy-pm-handle :controller-manager controller-manager
                         :right-arm right-arm-joint-controller
                         :left-arm left-arm-joint-controller
                         :left-arm-vel-mux left-arm-vel-mux
                         :right-arm-vel-mux right-arm-vel-mux)
))

(defun get-boxy-pm-handle ()
  ;; WARNING: HAS SIDE-EFFECTS
  (unless *boxy-pm-handle*
    (setf *boxy-pm-handle* (init-boxy-pm-handle)))
  *boxy-pm-handle*)

(defun cleanup-boxy-pm-handle (handle)
  (close-persistent-service (boxy-controller-manager handle)))

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(def-action-handler arm-joint-move (side config &optional (time 5.0))
  (ros-info (boxy-pm) "Moving ~a arm to config: ~a" side config)
  (let* ((handle (get-boxy-pm-handle))
         (controller (ecase side
                       (:left (boxy-left-arm handle))
                       (:right (boxy-right-arm handle))))
         (joint-names (ecase side
                        (:left (get-left-arm-joint-names))
                        (:right (get-right-arm-joint-names)))))
    (ensure-pos-controllers (boxy-controller-manager handle))
    (move-arm-config controller joint-names config time))
  (ros-info (boxy-pm) "Moving done."))

(def-process-module boxy-process-module (desig)
  (apply #'call-action (reference desig)))

(def-fact-group boxy-pm-action-designators (action-desig)

  (<- (action-desig ?designator (arm-joint-move ?side ?config))
    (desig::action-desig? ?designator)
    (desig-prop ?designator (:to :move))
    (desig-prop ?designator (:arm ?side))
    (desig-prop ?designator (:config ?config))))

(def-fact-group boxy-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator boxy-process-module)
    (or (format "2")
        (desig-prop ?designator (:to :home))
        (format "3")
        (desig-prop ?designator (:arm ?_))
        (format "4")
        (desig-prop ?designator (:config ?_)
                    )
        (format "5")))
                     
  (<- (available-process-module boxy-process-module)
    (crs:true)))