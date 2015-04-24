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

;; (defparameter *right-arm-home-config*
;;   (list -0.364994078874588 -0.2963586449623108 1.446787714958191 1.4938795566558838
;;         1.8305927515029907 1.9798997640609741 0.7551921010017395))

(defun move-arm-config (arm joint-names goal-config execution-time)
  (actionlib-lisp:send-goal-and-wait 
   arm
   (actionlib-lisp:make-action-goal-msg arm
     :trajectory (make-trajectory-msg joint-names goal-config execution-time))
   (+ execution-time 1.0)
   1.0))

(defun get-right-arm-joint-names ()
  (list "right_arm_0_joint" "right_arm_1_joint" "right_arm_2_joint" "right_arm_3_joint"
        "right_arm_4_joint" "right_arm_5_joint" "right_arm_6_joint"))

(defun get-left-arm-joint-names ()
  (list "left_arm_0_joint" "left_arm_1_joint" "left_arm_2_joint" "left_arm_3_joint"
        "left_arm_4_joint" "left_arm_5_joint" "left_arm_6_joint"))

;;;
;;; INTERNAL API
;;;

(defun make-trajectory-msg (joint-names goal-config execution-time)
  (make-msg
   "trajectory_msgs/JointTrajectory"
   :header (make-msg "std_msgs/Header" :stamp (ros-time))
   :joint_names (coerce joint-names 'vector)
   :points (coerce (list (make-trajectory-point goal-config execution-time)) 'vector)))


(defun make-trajectory-point (goal-config execution-time)
  (roslisp:make-msg 
   "trajectory_msgs/JointTrajectoryPoint"
   :positions (coerce goal-config 'vector)
   :time_from_start execution-time))