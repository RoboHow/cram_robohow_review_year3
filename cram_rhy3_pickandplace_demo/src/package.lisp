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

(in-package :cl-user)

(desig-props:def-desig-package cram-rhy3-pickandplace-demo
  (:use #:common-lisp #:roslisp #:cram-utilities #:designators-ros
        #:cram-roslisp-common #:cram-designators #:location-costmap
        #:cram-plan-knowledge #:cram-plan-library)
  (:export
   ;; Symbols
   pr2
   boxy)
  (:import-from :cram-language 
                top-level fl-funcall with-tags pursue tag retry-after-suspension
                whenever pulsed value with-task-suspended seq)
  (:import-from :cram-designators 
                make-designator 
                action
                action-desig? 
                desig-prop
                action-desig)
  (:import-from :robosherlock-process-module
                infer-object-property
                perceived-object-invalid
                object-handle)
  (:import-from :cram-language-designator-support with-designators)
  (:import-from :cram-language def-cram-function def-top-level-cram-function)
  (:import-from :cram-reasoning def-fact-group <- not)
  (:desig-properties #:distance #:around #:robot #:at #:goal
                     #:type #:on #:name #:to #:grasp #:obj
                     #:trajectory #:handle #:joint #:joint-axis
                     #:lower-bound #:upper-bound #:box #:name #:at
                     #:color #:size #:shape #:2d-range #:spatula
                     #:obstacle #:aware #:pancakemaker #:pancakemix
                     #:pancake #:side #:rightof #:leftof #:handle
                     #:max-handles #:top-slide-down #:behind #:in-front-of
                     #:left-of #:right-of #:for #:oven #:armarker #:id
                     #:dimensions #:pose #:tray #:island #:carry-handles
                     #:grasp-type #:black #:yellow #:white #:grey #:red
                     #:blu #:cyan #:magenta #:green #:timestamp
                     #:plane-distance #:segment #:dimensions-2d #:on
                     #:name #:response #:spoon #:tomato-sauce #:distance))
