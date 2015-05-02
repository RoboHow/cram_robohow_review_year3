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

(defgeneric switch-controllers (cm start stop)
  (:method ((cm roslisp::persistent-service) start stop)
    (with-fields (ok)
        (call-persistent-service 
         cm
         :start_controllers (map 'vector #'identity start)
         :stop_controllers (map 'vector #'identity stop)
         :strictness 1)
      (unless ok
        (error "Switching controllers failed. Start: ~a, stop: ~a." start stop))))
  (:method ((cm roslisp::service-client) start stop)
    (with-fields (ok)
        (call-service
         cm
         :start_controllers (map 'vector #'identity start)
         :stop_controllers (map 'vector #'identity stop)
         :strictness 1)
        (unless ok
          (error "Switching controllers failed. Start: ~a, stop: ~a." start stop)))))

(defgeneric get-pos-controller-names (name)
  (:method ((name (eql :right)))
    (list "r_arm_traj_controller"))
  (:method ((name (eql :left)))
    (list "l_arm_traj_controller"))
  (:method ((name (eql :both)))
    (concatenate 'list (get-pos-controller-names :left) (get-pos-controller-names :right))))

(defgeneric get-vel-controller-names (name)
  (:method ((name (eql :right)))
    (list "r_arm_vel"))
  (:method ((name (eql :left)))
    (list "l_arm_vel"))
  (:method ((name (eql :both)))
    (concatenate 'list (get-vel-controller-names :left) (get-vel-controller-names :right))))

(defun stop-controllers (cm &key (arms :both))
  (let ((pos-ctrls (get-pos-controller-names arms))
        (vel-ctrls (get-vel-controller-names arms)))
    (switch-controllers cm nil (concatenate 'list pos-ctrls vel-ctrls))))

(defun ensure-vel-controllers (cm &key (arms :both))
  (let ((pos-ctrls (get-pos-controller-names arms))
        (vel-ctrls (get-vel-controller-names arms)))
    (switch-controllers cm vel-ctrls pos-ctrls)))

(defun ensure-pos-controllers (cm &key (arms :both))
  (let ((pos-ctrls (get-pos-controller-names arms))
        (vel-ctrls (get-vel-controller-names arms)))
    (switch-controllers cm pos-ctrls vel-ctrls)))