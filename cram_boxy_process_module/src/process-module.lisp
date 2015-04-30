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

;;;
;;; HANDLE STUFF
;;;

(defparameter *boxy-pm-handle* nil)

(defstruct (boxy-pm-handle (:conc-name boxy-))
  (controller-manager nil)
  (right-arm nil)
  (left-arm nil)
  (right-arm-cart nil)
  (left-arm-cart nil)
  (left-arm-vel-mux nil)
  (right-arm-vel-mux nil)
  (left-gripper nil)
  (right-gripper nil)
  (ptu nil))
  
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
           "/l_arm_traj_controller/follow_joint_trajectory"
           "control_msgs/FollowJointTrajectoryAction"))
        (left-arm-vel-mux (init-mux-handle "/l_arm_vel/mux"))
        (right-arm-vel-mux (init-mux-handle "/r_arm_vel/mux"))
        (right-gripper (cram-wsg50:make-wsg50-handle "/right_arm_gripper"))
        (left-gripper (cram-wsg50:make-wsg50-handle "/left_arm_gripper"))
        (ptu (actionlib-lisp:make-simple-action-client
              "/ptu" "iai_control_msgs/PTUAction"))
        (left-arm-cart (init-cartesian-controller-handle
                        "/l_arm_cart_controller"))
        (right-arm-cart (init-cartesian-controller-handle
                        "/r_arm_cart_controller"))
)
    (make-boxy-pm-handle :controller-manager controller-manager
                         :right-arm right-arm-joint-controller
                         :left-arm left-arm-joint-controller
                         :left-arm-vel-mux left-arm-vel-mux
                         :right-arm-vel-mux right-arm-vel-mux
                         :left-gripper left-gripper
                         :right-gripper right-gripper
                         :ptu ptu
                         :left-arm-cart left-arm-cart
                         :right-arm-cart right-arm-cart
                         )
))

(defun get-boxy-pm-handle ()
  ;; WARNING: HAS SIDE-EFFECTS
  (unless *boxy-pm-handle*
    (setf *boxy-pm-handle* (init-boxy-pm-handle)))
  *boxy-pm-handle*)

(defun cleanup-boxy-pm-handle (handle)
  (close-persistent-service (boxy-controller-manager handle)))

;;;
;;; PROCESS MODULE GENERALITIES
;;;

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defun valid-package-symbol-p (symbol)
  "Predicate that checks whether `symbol' is interned in CRAM-BOXY-PM."
  (eql (symbol-package symbol) (find-package 'cram-boxy-process-module)))

(defun safe-reference (desig)
  "Iterates over all reference solutions for the action designator `desig' until
 it finds one that refers to an action-handler in this package and returns that
 solution. If no such solution exists, it returns NIL."
  (loop for solution = (reference des) with des = desig do
    (when (valid-package-symbol-p (car solution))
      (return solution))
    (if (next-solution des)
        (setf des (next-solution des))
        (return nil))))

(def-process-module boxy-process-module (desig)
  (apply #'call-action (safe-reference desig)))

;;;
;;; SPECIFIC ACTION-HANDLERS
;;;

(def-action-handler arm-joint-move (side config &optional (time 5.0))
 (arm-joint-move side config time))

(defun arm-joint-move (side config &optional (time 5.0))
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

(defun arm-cart-move (side goal-pose ee-frame &optional (timeout 5.0))
  (ros-info (boxy-pm) "Moving ~a arm to pose ~a" side goal-pose)
  (let* ((handle (get-boxy-pm-handle))
         (mux (ecase side
               (:left (boxy-left-arm-vel-mux handle))
               (:right (boxy-right-arm-vel-mux handle))))
         (controller (ecase side
                       (:left (boxy-left-arm-cart handle))
                       (:right (boxy-right-arm-cart handle))))
         (controller-out-topic (cart-out-topic controller))
         (state-fluent (cart-state-fluent controller)))
    (switch-mux mux controller-out-topic)
    (cpl-impl:pursue
      (cpl:sleep timeout)
      (cpl-impl:seq
        (move-cart controller goal-pose ee-frame)
        (cpl-impl:wait-for (cpl-impl:fl-funcall #'cart-controller-finished-p state-fluent)))
      (cpl-impl:whenever ((cpl:pulsed state-fluent))
        (move-cart controller goal-pose ee-frame)))))

(def-action-handler look-at (pose &optional (timeout 5.0))
  (ros-info (boxy-pm) "Looking at pose ~a" pose)
  (let* ((ptu (boxy-ptu (get-boxy-pm-handle))))
    (actionlib-lisp:send-goal-and-wait 
     ptu (actionlib-lisp:make-action-goal-msg ptu
           :mode 0 :pose (cl-tf:pose-stamped->msg pose))
     timeout 1.0)))  

(def-action-handler grasp-object (side pre-config)
  (format t "in grasping function~%")
  (let* ((handle (get-boxy-pm-handle))
         (gripper (ecase side
                   (:left (boxy-left-gripper handle))
                   (:right (boxy-right-gripper handle)))))
    (cram-wsg50:move-wsg50 gripper 110 60 20)
    (cpl::sleep 1)
    (arm-joint-move side pre-config)
    (cram-wsg50:move-wsg50 gripper 5 60 20)
    (cpl:sleep 3)))

;;;
;;; PROLOG FACTS
;;;

(def-fact-group boxy-pm-action-designators (action-desig)

  (<- (action-desig ?designator (arm-joint-move :right ?config)) 
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to park))
    (desig-prop ?designator (arm right))
    (equal ?config (-1.25 -1.23 -0.29 -2.1 0.4 0.58 0.13)))

  (<- (action-desig ?designator (arm-joint-move :left ?config)) 
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to park))
    (desig-prop ?designator (arm left))
    (equal ?config (-2.13 -0.93 0.45 -1.89 -0.15 0.1 1.3)))

  (<- (action-desig ?designator (look-at ?pose))
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to follow))
    (desig-prop ?designator (pose ?pose)))
      
  (<- (action-desig ?designator (grasp-object ?side ?pre-config))
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to grasp))
    (format "in grasping pattern~%")
    (desig-prop ?designator (obj ?obj))
    (format "a")
    (current-designator ?obj ?current-obj)
    (format "b")
    (obj-desig? ?current-obj)
    (format "c")
    (desig-prop ?current-obj (type spoon))
    (format "d")
    (equal ?side :right)
    (format "e")
    (equal ?pre-config (-1.47 0.98 -1.2 -1.9 0.26 0.0 1.1))
    (format "f"))
)

(def-fact-group boxy-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator boxy-process-module)
    (desig::action-desig? ?designator)
    (or 
     (and 
      (desig-prop ?designator (type trajectory))
      (desig-prop ?designator (to park))
      (desig-prop ?designator (arm ?_)))
     (and
      (desig-prop ?designator (type trajectory))
      (desig-prop ?designator (to follow))
      (desig-prop ?designator (pose ?pose)))
     (and
      (desig-prop ?designator (type trajectory))
      (desig-prop ?designator (to grasp))
      (desig-prop ?designator (obj ?_)))

))
                           
  (<- (available-process-module boxy-process-module)
    (crs:true)))