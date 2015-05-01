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

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(def-process-module lasa-process-module (desig)
  (apply #'call-action (reference desig)))

(def-action-handler lasa-perceive (object-designator)
  (ros-info (lasa-pm) "Perceiving dough with LASA perception")
  (with-fields ((size area) (dough-p dough_found)
                (object-frame object_frame) (reach-center reach_center_attractor)
                (reach-corner reach_corner_attractor) (roll-attractor roll_attractor)
                (back-attractor back_attractor))
      (call-lasa-perception (get-handle))
    (format t "hu~%")
    (if dough-p
        (copy-designator object-designator
                         :new-description 
                         `((size ,size) (object-frame ,object-frame) 
                           (reach-center ,reach-center) (reach-corner ,reach-corner) 
                           (roll-attractor ,roll-attractor) (back-attractor ,back-attractor)))
        (cpl:fail 'cram-plan-failures:object-not-found))))

(def-action-handler lasa-control (action-desig action-type action-name 
                                               object-frame attractor-frame force-gmm)
  (ros-info (lasa-pm) "Rolling dough with LASA controllers")
  (format t "desig: ~a~%type: ~a~%name: ~a~%object-frame: ~a~%attractor-frame: ~a~%force-gmm: ~a~%"
          action-desig action-type action-name object-frame attractor-frame force-gmm)
  (call-lasa-controller (get-handle) action-type action-name object-frame
                        attractor-frame force-gmm))

(def-fact-group lasa-process-module-action-designators (action-desig)

  (<- (lasa-pm-running?)
    (desig::lisp-pred get-running-process-module lasa-process-module))

  (<- (some-stuff-desig? ?desig)
    (obj-desig? ?desig)
    (desig-prop ?desig (some stuff)))

  (<- (action-desig ?desig (lasa-perceive ?current-desig))
    (lasa-pm-running?)
    (desig-prop ?desig (to perceive))
    (desig-prop ?desig (obj ?obj-desig))
    (current-designator ?obj-desig ?current-desig)
    (some-stuff-desig? ?current-desig))

  (<- (action-desig ?desig (lasa-control ?current-desig "LEARNED_MODEL" "reach" 
                                         ?object-frame ?attractor-frame ""))
    (lasa-pm-running?)
    (desig:trajectory-desig? ?desig)
    (current-designator ?desig ?current-desig)
    (desig-prop ?current-desig (to reach))
    (desig-prop ?current-desig (obj ?obj-desig))
    (current-designator ?obj-desig ?current-obj-desig)
    (some-stuff-desig? ?current-obj-desig)
    (desig-prop ?current-obj-desig (object-frame ?object-frame))
    (desig-prop ?current-desig (iteration ?iteration))
    (crs:once
     (or 
      (and
       (desig::lisp-pred < ?iteration 3)
       (desig-prop ?current-obj-desig (reach-center ?attractor-frame)))
      (desig-prop ?current-obj-desig (reach-corner ?attractor-frame)))))

  (<- (action-desig ?desig (lasa-control ?current-desig "LEARNED_MODEL" "roll" 
                                         ?object-frame ?attractor-frame ?force-gmm))
    (lasa-pm-running?)
    (desig:trajectory-desig? ?desig)
    (current-designator ?desig ?current-desig)
    (desig-prop ?current-desig (to roll))
    (desig-prop ?current-desig (obj ?obj-desig))
    (current-designator ?obj-desig ?current-obj-desig)
    (some-stuff-desig? ?current-obj-desig)
    (desig-prop ?current-obj-desig (object-frame ?object-frame))
    (desig-prop ?current-obj-desig (roll-attractor ?attractor-frame))
    (desig-prop ?current-obj-desig (size ?size))
    (crs:once
     (or
      (and (desig::lisp-pred < ?size 0.015) 
           (equal ?force-gmm "first"))
      (and (desig::lisp-pred < 0.015 ?size)
           (desig::lisp-pred < ?size 0.03)
           (equal ?force-gmm "mid"))
      (equal ?force-gmm "last"))))

  (<- (action-desig ?desig (lasa-control ?current-desig "LEARNED_MODEL" "back" 
                                         ?object-frame ?attractor-frame ""))
    (lasa-pm-running?)
    (desig:trajectory-desig? ?desig)
    (current-designator ?desig ?current-desig)
    (desig-prop ?current-desig (to retract))
    (desig-prop ?current-desig (obj ?obj-desig))
    (current-designator ?obj-desig ?current-obj-desig)
    (some-stuff-desig? ?current-obj-desig)
    (desig-prop ?current-obj-desig (object-frame ?object-frame))
    (desig-prop ?current-obj-desig (back-attractor ?attractor-frame)))
)

(def-fact-group lasa-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?desig lasa-process-module)
    (desig::action-desig? ?desig)
    (or (desig-prop ?desig (to perceive))
        (desig-prop ?desig (to reach))
        (desig-prop ?desig (to roll))
        (desig-prop ?desig (to retract))))

  (<- (available-process-module lasa-process-module)
    (crs:true)))