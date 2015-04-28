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

;;;
;;; Demo Plans
;;;

;; NOTE(winkler): An initial idea for letting the two robots
;; collaborate would be through the demo command topic
;; (`/demo_command'). There already is some infrastructure for this in
;; `utils.lisp' that allows to blockingly wait for a specific string
;; there, and send out strings. We could use this as a communication
;; backend to make the two robots wait for each other, i.e., respect
;; the current `phase' of the demo.

;; NOTE(winkler): Two top level plans, one for each robot. @georg, is
;; this feasible for you?
(def-top-level-cram-function pizza-making-pr2 ()
  (with-process-modules-pr2
    (let ((dh (get-demo-handle)))
      (initialize-demo-setup dh :pr2)
      ;; TODO(all): Perform actual PR2 demo plans here
      ;; Do everything pizza-making related here before moving the tray
      (fetch-tray dh)
      ;(shove-tray-into-oven dh)
      (send-kqml dh "PR2" "Boxy" "tray placed in oven")
      (destroy-demo-handle dh))))

(def-top-level-cram-function pizza-making-boxy ()
  (with-process-modules-boxy
    (let ((dh (get-demo-handle)))
      (initialize-demo-setup dh :boxy)
      ;; TODO(all): Perform actual Boxy demo plans here
      (human-tracking) ;; This happens after everything else and triggers tracking of the human. It automatically waits for the PR2 to report that the tray was shoven into the oven.
      (destroy-demo-handle dh))))

(def-cram-function grasp-spoon (demo-handle)
  (let ((park-right-arm-action
          (make-designator 'action '((:to :move)
                                     (:arm :right)
                                     (:config (-1.25 -1.23 -0.29 -2.1 0.4 0.58 0.13)))))
        (pregrasp-config-action
          (make-designator 'action '((:to :move)
                                     (:arm :right)
                                     (:config (-1.47 0.98 -1.2 -1.9 0.26 0.0 1.1))))))
    (perform park-right-arm-action)
    (perceive-spoon demo-handle)
    (perform pregrasp-config-action)))

(def-cram-function fetch-tray (demo-handle)
  (in-front-of-island demo-handle
    (try-forever
      (pick-object (perceive-tray demo-handle) :stationary t))))

(def-cram-function fetch-tomato-sauce (demo-handle)
  (let ((tomato-sauce (get-tomato-sauce demo-handle)))
    (put-tomato-sauce-on-table demo-handle tomato-sauce)))

(def-cram-function fetch-spoon (demo-handle)
  (let ((spoon (get-spoon demo-handle)))
    (put-spoon-on-table demo-handle spoon)))

(def-cram-function shove-tray-into-oven (demo-handle)
  (in-front-of-oven demo-handle
    (look-at-marker-suitable-pose)
    (let ((perceived-markers (ensure-results
                              (lambda ()
                                (perceive-markers demo-handle)))))
      (ros-info (rh demo) "Found ~a marker(s)~%"
                (length perceived-markers))
      (labels ((marker-relative-pose (relative-pose)
                 (markers-relative-pose->absolute-poses
                  demo-handle perceived-markers relative-pose)))
        (publish-pose (first (marker-relative-pose
                              (cl-transforms:make-identity-pose)))
                      "/marker")))))

(def-cram-function track-human (demo-handle)
  (wait-for-kqml-message
   demo-handle "PR2" "Boxy" "tray placed in oven")
  (with-logging-disabled
    (perceive-tracked-human demo-handle))
  (with-logging-enabled
    (loop until (wait-for-kqml-message
                 demo-handle
                 "PR2" "Boxy" "tray appeared on kitchen_island"))))

(def-cram-function wait-for-orders-after-human-tracking (demo-handle)
  (let ((kqml (wait-for-kqml-message
               demo-handle "Boxy" "PR2" "Come back to the table.")))
    (in-front-of-island demo-handle
      (reply-to-kqml demo-handle kqml "I am back."))))
