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

(defun object-color (colors color)
  (let ((color-pair (find color colors :test (lambda (x y) (eql x (car y))))))
    (if color-pair
        (cadr color-pair)
        0.0d0)))

(defun make-area-restriction-cost-function ()
  (let ((min-x -1.0)
        (max-x 1.5)
        (min-y 0.5)
        (max-y 1.7))
    (lambda (x y)
      (if (and (>= x min-x)
               (<= x max-x)
               (>= y min-y)
               (<= y max-y))
          (if (> x 0.25)
              (if (< y 1.0)
                  1.0d0
                  0.0d0)
              1.0d0)
          0.0d0))))

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'area-restriction-distribution2)))
  77)

(def-fact-group demo-costmap-desigs-area-restriction (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (desig-props:to desig-props:see))
        (desig-prop ?desig (desig-props:to desig-props:reach))
        (desig-prop ?desig (desig-props:on ?_)))
    (costmap ?cm)
    (costmap-add-function area-restriction-distribution2
                          (make-area-restriction-cost-function)
                          ?cm)))

(defun detection-has-type (detection)
  (cadr (assoc 'desig-props:type detection)))

(def-fact-group object-refinement-facts (infer-object-property
                                         object-handle
                                         cram-language::grasp-effort
                                         reorient-object)

  (<- (object-color ?object ?color ?value)
    (desig-prop ?object (desig-props:color ?colors))
    (crs:lisp-fun object-color ?colors ?color ?value))
  
  (<- (make-handles-further ?segments ?offset-angle ?handles)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.06 ?segments ?offset-angle desig-props:push
                  ?pi-half 0 0 0 0 0.05 ?handles))
  
  (<- (make-handles ?segments ?offset-angle ?handles)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.04 ?segments ?offset-angle desig-props:push
                  ?pi-half 0 0 0 0 0 ?handles))
  
  (<- (make-handles ?distance-from-center ?segments ?offset-angle ?grasp-type
                    ?hand-ax ?hand-ay ?hand-az ?co-x ?co-y ?co-z ?handles)
    (crs:lisp-fun cl-transforms:make-3d-vector ?co-x ?co-y ?co-z ?co)
    (crs:lisp-fun make-handles ?distance-from-center
                  :segments ?segments
                  :offset-angle ?offset-angle
                  :grasp-type ?grasp-type
                  :ax ?hand-ax
                  :ay ?hand-ay
                  :az ?hand-az
                  :center-offset ?co
                  ?handles))
  
  (<- (infer-object-property ?object desig-props:type desig-props:tray)
    ;;(object-color ?object desig-props:black ?black)
    ;;(> ?black 0.7))
    (desig-prop ?object (desig-props::detection ?detection))
    (crs:lisp-fun detection-has-type ?detection "tray"))

    ;;(desig-prop ?object (desig-props::shape "flat")))
  
  (<- (infer-object-property ?object desig-props:type desig-props:spoon)
    (desig-prop ?object (desig-props:response ?response))
    (crs:lisp-pred string= ?response "spoon"))
  
  (<- (infer-object-property ?object desig-props:type desig-props:tomato-sauce)
    (desig-prop ?object (desig-props::detection ?detection))
    (crs:lisp-fun detection-has-type ?detection "Ketchup_bottle"))
  
  (<- (infer-object-property ?object desig-props:handle ?handle)
    (crs:once
     (or (desig-prop ?object (desig-props:type ?type))
         (infer-object-property ?object desig-props:type ?type)))
    (crs:lisp-fun symbol-package ?type ?pkg)
    (object-handle ?type ?handles-list)
    (member ?handle ?handles-list))
  
  (<- (object-handle desig-props:tray ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 4 ?tilt1)
    (crs:lisp-fun * ?tilt1 5 ?tilt2)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.18 2 0 desig-props:push ?pi ?tilt2
                  0.0 0.0 0.0 0.0 ?handles-list))
  
  (<- (object-handle desig-props:tomato-sauce ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles-further 1 -0.4 ?handles-list))
  
  (<- (object-handle desig-props:spoon ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (crs:lisp-fun / ?pi -2 ?minus-pi-half)
    (make-handles 0.0 1 0 desig-props:push
                  ?pi-half ?minus-pi-half 0.0
                  0.0 0.0 -0.0 ?handles-list))
                  ;; 0.0 ?minus-pi-half ?pi-half
                  ;; ;; Fix these offsets! Maybe z is too low.
                  ;; 0.05 0.0 0.0 ?handles-list))
  
  ;; Tray: Carry with 2 arms
  (<- (object-carry-handles desig-props:tray 2))
  
  (<- (infer-object-property ?object desig-props:carry-handles ?carry-handles)
    (desig-prop ?object (type ?type))
    (object-carry-handles ?type ?carry-handles)))
