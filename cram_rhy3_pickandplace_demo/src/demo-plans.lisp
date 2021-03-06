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

(def-top-level-cram-function pizza-making-pr2 (&key (human-tracking t)
                                                    (spoon t)
                                                    (tray t)
                                                    (ketchup t))
  (with-process-modules-pr2
    (let ((dh (get-demo-handle)))
      (initialize-demo-setup dh :pr2 :enable-logging t)
      (ensure-arms-up)
      (wait-for-external-trigger :force t)
      (select-rs-instance "handles")
      (when spoon
        (with-kqml-sent dh "PR2" "*" "fetching spoon" nil
          (fetch-spoon dh)))
      (when ketchup
        (with-kqml-sent dh "PR2" "*" "fetching tomato sauce" nil
          (fetch-tomato-sauce dh)))
      (when tray
        (wait-for-external-trigger :force t)
        (select-rs-instance "tray")
        (with-kqml-sent dh "PR2" "*" "fetching tray" nil
          (fetch-tray dh))
        (with-kqml-sent dh "PR2" "*" "bringing tray to oven" nil
          (tray-into-oven dh))
        (send-kqml dh "PR2" "Boxy" "tray placed on oven lid"))
      (when human-tracking
        (wait-for-external-trigger :force t)
        (back-off)
        (go-in-front-of-island-2)
        (go-in-front-of-fridge)
        (human-perception dh))
      (destroy-demo-handle dh))))

(def-cram-function human-perception (demo-handle)
  (let ((log-id (beliefstate:start-node "WAITING-FOR-HUMAN")))
    (with-kqml-sent
        demo-handle
        "Boxy" "PR2"
        "i am waiting for a human to put the tray into the oven" nil
      (wait-for-human-in-scene demo-handle))
    (beliefstate:stop-node log-id))
  (let ((log-id (beliefstate:start-node "SEEING-HUMAN")))
    (with-kqml-sent demo-handle "Boxy" "PR2"
        "a human entered the scene" nil
      (wait-while-human-in-scene demo-handle))
    (beliefstate:stop-node log-id))
  (let ((log-id (beliefstate:start-node "WITH-HUMAN-LEFT")))
    (with-kqml-sent demo-handle "Boxy" "PR2"
        "the human left the scene again" nil
      (sleep 5))
    (beliefstate:stop-node log-id)))

(def-top-level-cram-function pizza-making-boxy ()
  (with-process-modules-boxy
    (let ((dh (get-demo-handle)))
      (initialize-demo-setup dh :boxy)
      (select-rs-instance "pizza_demo")
      (wait-for-kqml-message dh "PR2" "Boxy" "spoon on counter")
      (grasp-spoon)
      (spread-tomato-sauce)
      ;; TODO(all): Perform actual Boxy demo plans here

      ;(human-tracking) ;; This happens after everything else and triggers tracking of the human. It automatically waits for the PR2 to report that the tray was shoven into the oven.
      (destroy-demo-handle dh))))

(def-cram-function grasp-spoon ()
  (with-designators ((spoon (object '((desig-props::type desig-props::spoon))))
                     (park-right-arm (action '((desig-props::type desig-props::trajectory)
                                               (desig-props::to desig-props::park)
                                               (desig-props::arm desig-props::right))))
                     (park-left-arm (action '((desig-props::type desig-props::trajectory)
                                               (desig-props::to desig-props::park)
                                               (desig-props::arm desig-props::left))))
   )
 ;  (perform park-right-arm)
 ;  (perform park-left-arm)
    ;; lookat right-table
    (equate spoon (first (perceive-object 'cram-plan-library:currently-visible spoon)))
    (perform (make-designator 'action `((desig-props::type desig-props::trajectory) 
                                        (desig-props::to desig-props::grasp) 
                                        (desig-props::obj ,spoon))))
;      (achieve `(object-picked ,spoon))
      ;; lookat center
      ;; perceive pizza tray
      ;; perceive tomate sauce
      ;; add tomate sauce
      ;; perceive cheese
      ;; add cheese
      ;; lookat left side
      ;; perceive ham
      ;; add ham
      ;;
      ;; call pr2
      ))

(def-cram-function spread-tomato-sauce ()
  (with-designators ((tray (object '((desig-props::type desig-props::tray))))
                     (tomato-bowl (object '((desig-props::type desig-props::red_bowl)))))
    (equate tray (first (perceive-object 'cram-plan-library:currently-visible tray)))
    (equate tomato-bowl (first (perceive-object 'cram-plan-library:currently-visible tomato-bowl)))
    (with-designators ((add-tomato 
                        (action `((desig-props::type desig-props::trajectory)
                                  (desig-props::to desig-props::add)
                                  (desig-props::stuff desig-props::tomato)
                                  (desig-props::source ,tomato-bowl)
                                  (desig-props::destination ,tray)))))
      (perform add-tomato)
      tomato-bowl
      ;; to be continued...
      )))

(def-cram-function fetch-tray (demo-handle)
  (ensure-arms-up)
  (in-front-of
      (desig:make-designator
       'location
       `((desig-props::pose
          ,(cl-tf:pose->pose-stamped
            "/map" 0.0
            (cl-transforms:make-pose
             (cl-transforms:make-3d-vector -0.354 1.4 0.0)
             (cl-transforms:make-quaternion 0 0 1 0))))
         (desig-props:in-front-of desig-props:island)))
      t)
  (look-onto-island)
  (let ((tray (ensure-results (perceive-tray demo-handle))))
    (try-forever
      (pick-object tray :stationary t))))

(def-cram-function fetch-tomato-sauce (demo-handle)
  (let ((tomato-sauce (get-tomato-sauce demo-handle)))
    (put-tomato-sauce-on-table demo-handle tomato-sauce)))

(def-cram-function fetch-spoon (demo-handle)
  (let ((spoon (get-spoon demo-handle)))
    (put-spoon-on-table demo-handle spoon)))

(def-cram-function shove-tray-into-oven (demo-handle)
  (in-front-of-oven demo-handle)
  (lift-arms-to 1.2))
  ;; (look-at-marker-suitable-pose)
  ;; (let ((perceived-markers (ensure-results
  ;;                            (lambda ()
  ;;                              (perceive-markers demo-handle)))))
  ;;   (ros-info (rh demo) "Found ~a marker(s)~%"
  ;;             (length perceived-markers))
  ;;   (labels ((marker-relative-pose (relative-pose)
  ;;              (markers-relative-pose->absolute-poses
  ;;               demo-handle perceived-markers relative-pose)))
  ;;     (publish-pose (first (marker-relative-pose
  ;;                           (cl-transforms:make-identity-pose)))
  ;;                   "/marker"))))

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
