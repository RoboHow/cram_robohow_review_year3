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

(in-package :cram-rolling-demo)

(defstruct (rolling-demo-handle (:conc-name dh-))
  (boxy-pm nil)
  (lasa-perception nil)
  (lasa-controllers nil))

(defun init-rolling-demo ()
  (let ((boxy-pm (boxy-pm:init-boxy-pm-handle))
        (lasa-perception 
          (make-instance 'persistent-service
                         :service-name "/attractor_pose"
                         :service-type "lasa_perception_module/attractorPose"))
        (lasa-controllers
          (actionlib-lisp:make-simple-action-client
           "plan2ctrl"
           "lasa_action_planners/PLAN2CTRLAction")))
    (actionlib-lisp:wait-for-server lasa-controllers 2.0)
    (make-rolling-demo-handle
     :boxy-pm boxy-pm
     :lasa-perception lasa-perception
     :lasa-controllers lasa-controllers)))

(defun call-lasa-perception (handle)
  (declare (ignore handle))
  (call-service "/attractor_pose" "lasa_perception_module/attractorPose")
  ;; (call-persistent-service (dh-lasa-perception handle))
  )

(defun call-lasa-controller (handle action-type action-name object-frame attractor-frame
                             force-gmm-id)
  (actionlib-lisp:send-goal-and-wait
   (dh-lasa-controllers handle)
   (actionlib-lisp:make-action-goal-msg (dh-lasa-controllers handle)
     :action_type action-type
     :action_name action-name
     :object_frame object-frame
     :attractor_frame attractor-frame
     :force_gmm_id force-gmm-id)
   0.0
   1.0))

(defparameter *handle* nil)

(defun get-handle ()
  (unless *handle*
    (setf *handle* (init-rolling-demo)))
  *handle*)