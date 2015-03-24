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

(def-fact-group object-refinement-facts (infer-object-property
                                         object-handle
                                         cram-language::grasp-effort
                                         reorient-object)

  (<- (make-handles ?segments ?offset-angle ?handles)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.04 ?segments ?offset-angle 'desig-props::push
                  ?pi-half 0 0 0 0 0 ?handles))
  
  (<- (make-handles ?distance-from-center ?segments ?offset-angle ?grasp-type
                    ?hand-ax ?hand-ay ?hand-az ?co-x ?co-y ?co-z ?handles)
    (crs:lisp-fun tf:make-3d-vector ?co-x ?co-y ?co-z ?co)
    (crs:lisp-fun make-handles ?distance-from-center
                  :segments ?segments
                  :offset-angle ?offset-angle
                  :grasp-type ?grasp-type
                  :ax ?hand-ax
                  :ay ?hand-ay
                  :az ?hand-az
                  :center-offset ?co
                  ?handles))
  
  (<- (reorient-object ?object t)
    (desig-prop ?object (desig-props::type desig-props::bowl)))
  
  (<- (infer-object-property ?object desig-props:type desig-props::tray)
    (object-color ?object desig-props:black ?black)
    (> ?black 0.7))
  
  (<- (infer-object-property ?object desig-props:handle ?handle)
    (crs:once
     (or (desig-prop ?object (desig-props:type ?type))
         (infer-object-property ?object desig-props:type ?type)))
    (crs:lisp-fun symbol-package ?type ?pkg)
    (object-handle ?type ?handles-list)
    (member ?handle ?handles-list))
  
  (<- (object-handle desig-props::tray ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 4 ?tilt1)
    (crs:lisp-fun * ?tilt1 5 ?tilt2)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.18 2 0 'desig-props::push ?pi ?tilt2
                  0.0 0.0 0.0 0.0 ?handles-list))
  
  (<- (infer-object-property ?object desig-props::carry-handles ?carry-handles)
    (infer-object-property ?object desig-props:type ?type)
    (object-carry-handles ?type ?carry-handles))
  
  ;; Tray: Carry with 2 arms
  (<- (object-carry-handles desig-props::tray 2)))
