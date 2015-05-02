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

(defstruct (mux-handle (:conc-name mux-))
  (add nil)
  (select nil)
  (list nil)
  (namespace nil))

(defun init-mux-handle (namespace)
  "Creates a new mux-handle which offers its services in `namespace'."
  (let ((mux-add
          (make-instance
           'persistent-service
           :service-name (concatenate 'string namespace "/add")
           :service-type "topic_tools/MuxAdd"))
        (mux-select
          (make-instance
           'persistent-service
           :service-name (concatenate 'string namespace "/select")
           :service-type "topic_tools/MuxSelect"))
        (mux-list
          (make-instance
           'persistent-service
           :service-name (concatenate 'string namespace "/list")
           :service-type "topic_tools/MuxList")))
    (make-mux-handle :add mux-add :select mux-select :list mux-list
                     :namespace namespace)))
        
(defun mux-has-topic-p (handle in-topic)
  "Predicate to check whether the mux behind `handle' already subscribes
 to `in-topic'."
  (with-fields (topics) 
      (call-service 
       (concatenate 'string (mux-namespace handle) "/list") 
       "topic_tools/MuxList")
    (find in-topic (coerce topics 'list) :test #' string=)))

(defun ensure-mux-has-topic (handle in-topic)
  "Ensures that the mux behind `handle' subscribes to `in-topic'."
  (unless (mux-has-topic-p handle in-topic)
    (call-service 
     (concatenate 'string (mux-namespace handle) "/add") 
     "topic_tools/MuxAdd" :topic in-topic)))

(defun switch-mux (handle in-topic)
  "Switches the input topic of the mux behind `handle' to `in-topic'."
  (ensure-mux-has-topic handle in-topic)
  (call-service 
   (concatenate 'string (mux-namespace handle) "/select") 
   "topic_tools/MuxSelect" :topic in-topic))

(defun disable-mux (handle)
  (call-service 
   (concatenate 'string (mux-namespace handle) "/select") 
   "topic_tools/MuxSelect" :topic "__none"))
