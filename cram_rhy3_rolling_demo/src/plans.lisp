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

(def-top-level-cram-function rolling-demo (max-iterations target-size)
  (with-designators ((dough (object '((some stuff)))))
    (with-process-modules-running (lasa-process-module)
      (cpl:with-retry-counters ((retry-count max-iterations))
        (cpl:with-failure-handling
            (((or cram-plan-failures:object-not-found
                  cram-plan-failures:manipulation-failure
                  cram-plan-failures:location-not-reached-failure) (f)
               (declare (ignore f))
               (cpl:do-retry retry-count (cpl:retry))))
          (equate dough (plan-lib:perceive-object 'plan-lib:currently-visible dough))
          (roll-object dough (- max-iterations (cpl-impl:get-counter retry-count)))
          (equate dough (plan-lib:perceive-object 'plan-lib:currently-visible dough))
          (when (< (desig-prop-value (current-desig dough) 'size) target-size)
            (cpl:fail 'cram-plan-failures:manipulation-failure)))))))

(def-cram-function roll-object (object iteration)
  (with-designators 
      ((reach-action 
        (action `((type trajectory) (to reach) (obj ,object) (iteration ,iteration))))  
       (roll-action 
        (action `((type trajectory) (to roll) (obj ,object) (iteration ,iteration))))
       (retract-action 
        (action `((type trajectory) (to retract) (obj ,object) (iteration ,iteration)))))
    (plan-lib:perform reach-action)
    (plan-lib:perform roll-action)
    (plan-lib:perform retract-action)))

(defun main ()
  "Call this function from a bash-script to run the demo."
  (roslisp-utilities:startup-ros)
  (beliefstate:enable-logging t)
  (beliefstate:set-metadata 
   :robot "BOXY" 
   :creator"IAI, UniHB and LASA, EPFL"
   :experiment "Dough rolling experiment with LASA controllers for RoboHow Year3 review."
   :description "One armed dough rolling with learned GMM controllers, triggered by CRAM.")
  (rolling-demo)
  (beliefstate:extract-files)
  (roslisp-utilities:shutdown-ros))

;;;
;;; ORIGINAL SCRIPT
;;;

;; (defun main ()
;;   (let ((desired-area 0.2)
;;         (area 0.0)
;;         (perception-counter 0)
;;         (roll-counter 0)
;;         (max-perception-counter 10)
;;         (home-config '(-0.364994078874588 -0.2963586449623108 1.446787714958191 
;;                        1.4938795566558838 1.8305927515029907 1.9798997640609741 
;;                        0.7551921010017395)))
;;     (format t "ensuring pos controllers: ~a~%"
;;             (boxy-pm:ensure-pos-controllers 
;;              (boxy-pm:boxy-controller-manager (dh-boxy-pm (get-handle))) :arms :right))
;;     (format t "going into start config: ~a~%"
;;             (boxy-pm:move-arm-config 
;;              (boxy-pm:boxy-right-arm (dh-boxy-pm (get-handle)))
;;              (boxy-pm:get-right-arm-joint-names)
;;              home-config 5.0))
;;     (loop while (and (< perception-counter max-perception-counter) 
;;                      (< area desired-area)) 
;;           do
;;       (format t "Iteration: ~a Area: ~a~%" perception-counter area)
;;       (with-fields ((area-val area) (dough-p dough_found)
;;                     (object-frame object_frame) (reach-center reach_center_attractor)
;;                     (reach-corner reach_corner_attractor) (roll-attractor roll_attractor)
;;                     (back-attractor back_attractor))
;;           (call-lasa-perception (get-handle))
;;         (when dough-p
;;           (setf area area-val)
;;           (format t "Found some dough~%")
;;           (let ((force-gmm-id 
;;                   (cond
;;                     ((< area-val 0.015) "first")
;;                     ((and (> area-val 0.015) (< area-val 0.03)) "mid")
;;                     ((>= area-val 0.03) "last")
;;                     (t (error "Got bad area value for dough size: ~a" area-val))))
;;                 (reach-attractor
;;                   (if (< roll-counter 3)
;;                       (progn
;;                         (format t "CHOOSING CENTER ATTRACTOR~%")
;;                         reach-center)
;;                       (progn
;;                         (format t "CHOOSING CORNER ATTRACTOR~%")
;;                         reach-corner))))
;;             (format t "ensure-vel-controllers: ~a~%"
;;                     (boxy-pm:ensure-vel-controllers 
;;                      (boxy-pm:boxy-controller-manager (dh-boxy-pm (get-handle))) :arms :right))
;;             (format t "reach: ~a~%"
;;                     (call-lasa-controller (get-handle)
;;                                   "LEARNED_MODEL"
;;                                   "reach"
;;                                   object-frame
;;                                   reach-attractor
;;                                   "")
;;                     )
;;             (format t "roll: ~a~%"
;;                     (call-lasa-controller (get-handle)
;;                                    "LEARNED_MODEL"
;;                                    "roll"
;;                                    object-frame
;;                                    roll-attractor
;;                                    force-gmm-id)
;;                     )
;;             (format t "back~%~a~%"
;;                     (call-lasa-controller (get-handle)
;;                                           "LEARNED_MODEL"
;;                                           "back"
;;                                           object-frame
;;                                           back-attractor
;;                                           "")
;;                     )
;;             (incf roll-counter)))
;;         (incf perception-counter))
;;     (format t "FINISH~%Area: ~a~%" area))))

;(call-lasa-controller *handle*
;                                         "learned_model"
;                                         "reach"/"roll"/"back"
;                                         object-frame-from-perception
;                                         reach/roll/back
;                                         ""/"first"/mid"/"last"