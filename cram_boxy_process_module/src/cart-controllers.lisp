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

(defstruct (cartesian-controller-handle (:conc-name cart-))
  (state-sub nil)
  (state-fluent nil)
  (command-pub nil)
  (out-topic nil))

(defun init-cartesian-controller-handle (controller-namespace)
  (let* ((state-fluent (cpl-impl:make-fluent))
         (state-topic (concatenate 'string controller-namespace "/state"))
         (in-topic (concatenate 'string controller-namespace "/in_command"))
         (out-topic (concatenate 'string controller-namespace "/out_command"))
         (state-sub (subscribe state-topic "iai_control_msgs/CartState"
                               (alexandria:curry #'cartesian-state-callback state-fluent)))
         (command-pub (advertise in-topic "iai_control_msgs/CartGoal")))
    (make-cartesian-controller-handle
     :state-sub state-sub :state-fluent state-fluent :command-pub command-pub :out-topic out-topic)))

(defun cartesian-state-callback (fluent msg)
  (flet ((msg->list (msg)
           (with-fields (running error error_pos error_rot vel_norm
                                 elapsed_seconds ee_frame_name base_frame_name) msg
             (list :running running :error error :error-pos error_pos :error-rot error_rot
                   :vel-norm vel_norm :elapsed-seconds elapsed_seconds
                   :ee-frame-name ee_frame_name :base-frame-name base_frame_name))))
    (setf (cpl-impl:value fluent) (msg->list msg))))

(defun move-cart (handle goal-pose ee-frame)
  (publish (cart-command-pub handle)
           (make-message 
            "iai_control_msgs/CartGoal"
            :pose (cl-tf:pose-stamped->msg goal-pose)
            :ee_frame_name ee-frame)))

(defun cart-controller-finished-p (controller-state
                                   &optional (vel-thresh 0.1) (pos-error-thresh 0.03))
  (and (getf controller-state :running)
       (< (getf controller-state :error-pos) pos-error-thresh)
       (< (getf controller-state :vel-norm) vel-thresh)))
