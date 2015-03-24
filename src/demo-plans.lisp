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
      (initialize-demo-setup dh 'pr2)
      ;; TODO(all): Perform actual PR2 demo plans here
      ;; Do everything pizza-making related here before moving the tray
      (fetch-tray dh)
      ;(shove-tray-into-oven dh)
      (destroy-demo-handle dh))))

(def-top-level-cram-function pizza-making-boxy ()
  (with-process-modules-boxy
    (let ((dh (get-demo-handle)))
      (initialize-demo-setup dh 'boxy)
      ;; TODO(all): Perform actual Boxy demo plans here
      (destroy-demo-handle dh))))

(def-cram-function fetch-tray (demo-handle)
  (in-front-of-island demo-handle
    (try-forever
      (pick-object (dh-obj-tray demo-handle) :stationary t))))

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
                              (tf:make-identity-pose)))
                      "/marker")))))
