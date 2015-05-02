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
  (ptu nil)
  (tf-broadcaster nil))
  
(defun init-boxy-pm-handle ()
  (let ((controller-manager 
          (make-service-client
           "/controller_manager/switch_controller" 
           "controller_manager_msgs/SwitchController"))
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
        (tf-broadcaster (cl-tf2:make-transform-broadcaster))
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
                         :tf-broadcaster tf-broadcaster
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
         (state-fluent (cart-state-fluent controller)))
    (switch-mux mux (cart-out-topic controller))
    (ensure-vel-controllers (boxy-controller-manager handle))
    (cpl-impl:pursue
      (cpl:sleep timeout)
      (cpl-impl:wait-for (cpl-impl:fl-funcall #'cart-controller-finished-p state-fluent))
      (cpl-impl:whenever ((cpl:pulsed state-fluent))
        (move-cart controller goal-pose ee-frame)))
    (stop-cartesian-controller controller)))

(def-action-handler look-at (pose &optional (timeout 5.0))
  (ros-info (boxy-pm) "Looking at pose ~a" pose)
  (let* ((ptu (boxy-ptu (get-boxy-pm-handle))))
    (actionlib-lisp:send-goal-and-wait 
     ptu (actionlib-lisp:make-action-goal-msg ptu
           :mode 0 :pose (cl-tf:pose-stamped->msg pose))
     timeout 1.0)))  

(defun publish-pose-stamped (pose-stamped child-frame-id)
  (with-slots ((frame-id cl-tf-datatypes:frame-id)
               (stamp cl-tf-datatypes:stamp)
               (orientation cl-tf-datatypes:orientation)
               (origin cl-tf-datatypes:origin)) pose-stamped
    (cl-tf2:send-transform
     (boxy-tf-broadcaster (get-boxy-pm-handle))
     (cl-tf2:make-stamped-transform
      frame-id
      child-frame-id
      (ros-time)
      (cl-transforms:make-transform
       origin orientation)))))

(def-action-handler grasp-object (side pre-config object-pose)
  (let* ((handle (get-boxy-pm-handle))
         (gripper (ecase side
                   (:left (boxy-left-gripper handle))
                   (:right (boxy-right-gripper handle))))
         (object-pose-in-base-link
           (cl-tf:transform-pose 
            cram-roslisp-common:*tf* 
            :pose (cl-tf-datatypes:copy-pose-stamped object-pose :stamp 0.0)
            :target-frame "base_link"))
         (goal-pose
           (cl-tf-datatypes:copy-pose-stamped
            object-pose-in-base-link
            ;; slightly alter position
            :origin 
            (cl-transforms:v+
             (cl-tf-datatypes:origin object-pose-in-base-link)
             (cl-transforms:make-3d-vector -0.025 0.0 0.0))
             ;; replace orientation with fixed one from Alexis
            :orientation (cl-transforms:make-quaternion 0.598 -0.588 0.387 -0.383)
            :stamp 0.0))
         (pre-pose
           (cl-tf-datatypes:copy-pose-stamped
            goal-pose
            :origin
            (cl-transforms:v+
             (cl-tf-datatypes:origin goal-pose)
             (cl-transforms:make-3d-vector 0.0 0.0 0.2)))))
    (cram-wsg50:move-wsg50 gripper 110 60 20)
    (arm-joint-move side pre-config)
    (publish-pose-stamped object-pose "spoon")
    (publish-pose-stamped pre-pose "spoon-pre-goal")
    (publish-pose-stamped goal-pose "spoon-goal")
    (arm-cart-move side pre-pose "right_gripper_tool_frame" 12.0)
    (cpl-impl:sleep* 0.5)
    (arm-cart-move side goal-pose "right_gripper_tool_frame" 12.0)
    (cram-wsg50:move-wsg50 gripper 5 60 20)
    (cpl:sleep 3)))

(def-action-handler add-stuff (side pre-config source-traj dest-traj)
  (declare (ignore pre-config dest-traj))
  (ros-info (boxy-pm) "Adding stuff with ~a arm" side)
  (loop for pose-stamped in source-traj with counter = 1 do
    (publish-pose-stamped 
     pose-stamped (concatenate 'string "SOURCE_" (write-to-string counter)))
    (incf counter))
)

(defun transform-pose-stamped (stamped-transform pose-stamped)
  (with-slots ((frame-id cl-tf-datatypes:frame-id)
               (stamp cl-tf-datatypes:stamp)) stamped-transform
    (with-slots ((orientation cl-tf-datatypes:orientation)
                 (origin cl-tf-datatypes:origin))
        (cl-transforms:transform-pose stamped-transform pose-stamped)
    (cl-tf-datatypes:make-pose-stamped frame-id stamp origin orientation))))

(defun pose-stamped->stamped-transform (pose-stamped child-frame-id)
  (with-slots ((frame-id cl-tf-datatypes:frame-id)
               (stamp cl-tf-datatypes:stamp)
               (orientation cl-tf-datatypes:orientation)
               (origin cl-tf-datatypes:origin)) pose-stamped
    (cl-tf-datatypes:make-stamped-transform 
     frame-id child-frame-id stamp origin orientation)))
        
(defun copy-stamped-transform (transform &key frame-id child-frame-id stamp
                                           translation rotation)
  (with-slots ((old-frame-id cl-tf-datatypes:frame-id)
               (old-child-frame-id cl-tf-datatypes:child-frame-id)
               (old-stamp cl-tf-datatypes:stamp)
               (old-translation cl-tf-datatypes:translation)
               (old-rotation cl-tf-datatypes:rotation)) transform
    (cl-tf-datatypes:make-stamped-transform 
     (or frame-id old-frame-id)
     (or child-frame-id old-child-frame-id)
     (or stamp old-stamp)
     (or translation old-translation)
     (or rotation old-rotation))))

(defun add-tomato-sauce-trajectory (source-transform dest-transform)
  (declare (ignore dest-transform))
  (let ((source-poses
          (list
           (cl-tf-datatypes:make-pose-stamped
            "RED_BOWL" 0.0
            (cl-transforms:make-3d-vector -0.11 -0.076 0.028)
            (cl-transforms:make-quaternion -0.664584 -0.552654 -0.27083 -0.42373)))))
    (list
     (mapcar 
      (lambda (source-pose)
        (cl-tf-datatypes:copy-pose-stamped
         (cl-tf:transform-pose 
          cram-roslisp-common:*tf* 
          :pose (transform-pose-stamped source-transform source-pose)
          :target-frame "base_link")
         :orientation (cl-transforms:make-identity-rotation)))
      source-poses)
     nil)
))

;;;
;;; PROLOG FACTS
;;;

(def-fact-group boxy-pm-action-designators (action-desig)

  (<- (object-desig-transform ?desig ?transform)
    (desig::obj-desig? ?desig)
    (current-designator ?desig ?current-obj)
    (desig-prop ?current-obj (at ?obj-loc))
    (desig-prop ?current-obj (type ?obj-type))
    (lisp-fun string ?obj-type ?obj-name)
    (desig::loc-desig? ?obj-loc)
    (current-designator ?obj-loc ?current-obj-loc)
    (desig-prop ?current-obj-loc (pose ?pose))
    (lisp-fun pose-stamped->stamped-transform ?pose ?obj-name ?transform))

  (<- (boxy-pm-running?)
    (desig::lisp-pred cram-process-modules::get-running-process-module
                      boxy-process-module))

  (<- (action-desig ?designator (arm-joint-move :right ?config)) 
    (boxy-pm-running?)
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to park))
    (desig-prop ?designator (arm right))
    (equal ?config (-1.25 -1.23 -0.29 -2.1 0.4 0.58 0.13)))

  (<- (action-desig ?designator (arm-joint-move :left ?config)) 
    (boxy-pm-running?)
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to park))
    (desig-prop ?designator (arm left))
    (equal ?config (-2.13 -0.93 0.45 -1.89 -0.15 0.1 1.3)))

  (<- (action-desig ?designator (look-at ?pose))
    (boxy-pm-running?)
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to follow))
    (desig-prop ?designator (pose ?pose)))
      
  (<- (action-desig ?designator (grasp-object ?side ?pre-config ?obj-pose))
    (boxy-pm-running?)
    (desig::action-desig? ?designator)
    (desig-prop ?designator (type trajectory))
    (desig-prop ?designator (to grasp))
    (format "in grasping pattern~%")
    (desig-prop ?designator (obj ?obj))
    (current-designator ?obj ?current-obj)
    (format "~a~%" ?current-obj)
    (obj-desig? ?current-obj)
    (desig-prop ?current-obj (type spoon))
    (desig-prop ?current-obj (at ?obj-loc))
    (current-designator ?obj-loc ?current-obj-loc)
    (desig-prop ?current-obj-loc (pose ?obj-pose))
    (equal ?side :right)
    (equal ?pre-config (-1.47 0.98 -1.2 -1.9 0.26 0.0 1.1))

)

  (<- (action-desig ?desig (add-stuff :right ?pre-config ?source-traj ?dest-traj))
    (boxy-pm-running?)
    (desig::trajectory-desig? ?desig)
    (desig-prop ?desig (to add))
    (desig-prop ?desig (stuff tomato))
    (desig-prop ?desig (source ?source-obj))
    (desig-prop ?desig (destination ?dest-obj))
    (object-desig-transform ?dest-obj ?dest-transform)
    (object-desig-transform ?source-obj ?source-transform)
    (equal ?pre-config (-1.97 0.8 -1.29 -1.05 0.52 1.29 -0.93))
    (lisp-fun add-tomato-sauce-trajectory ?source-transform ?dest-transform (?source-traj ?dest-traj))
)
)

(def-fact-group boxy-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?desig boxy-process-module)
    (desig::trajectory-desig? ?desig))
                           
  (<- (available-process-module boxy-process-module)
    (crs:true)))
