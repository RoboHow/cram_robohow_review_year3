;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Universität Bremen, nor the names of its
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

(in-package :cl-user)

(desig-props:def-desig-package robosherlock-mini-process-module
  (:nicknames robosherlock-mini-pm)
  (:use #:common-lisp
        #:roslisp
        #:cram-process-modules
        #:roslisp-utilities
        #:desig
        #:designators-ros)
  (:import-from #:cram-roslisp-common *tf2*)
  (:import-from #:cram-reasoning #:<- #:def-fact-group)
  (:import-from #:cram-manipulation-knowledge
                trajectory-point end-effector-link)
  (:export robosherlock-mini-process-module
           infer-object-property
           perceived-object-invalid
           object-handle)
  (:desig-properties #:to #:perceive #:obj #:examine #:property #:properties
                     #:color #:size #:shape #:pose-on-plane #:z-offset
                     #:grasp-pose #:grasp-poses #:object-identifier #:dimensions
                     #:pose #:identifier #:grasp-type #:unknown #:spatula
                     #:pancakemaker #:milkbox #:pancakemix #:scene
                     #:cylinder #:box #:yellow #:pancake #:pose-center
                     #:colors #:height #:width #:depth #:bb-pose
                     #:black #:white #:grey #:red #:blue #:magenta #:green
                     #:cyan #:flat #:round #:plane-distance #:at
                     #:resolution #:dimensions-3d #:dimensions-2d
                     #:boundingbox #:objectid #:lastseen #:name
                     #:segment #:on))
