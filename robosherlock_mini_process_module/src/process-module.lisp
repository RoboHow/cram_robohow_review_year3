;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :robosherlock-mini-process-module)

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defun post-process-results (results)
  (labels ((sub-value (name sequence)
             (cadr (find name sequence :test (lambda (x y)
                                               (eql x (car y))))))
           (process-object (property)
             (destructuring-bind (key value) property
               (cond ((eql key 'color)
                      `(,key ,value))
                     ((eql key 'type)
                      `(,key ,(intern (string-upcase value) 'desig-props)))
                     ((eql key 'shape)
                      `(,key ,(intern (string-upcase value) 'desig-props)))
                     ((eql key 'boundingbox)
                      `(,key
                        ,(mapcar
                          (lambda (resolution-property)
                            (destructuring-bind
                                (key value) resolution-property
                              (cond ((eql key 'dimensions-3d)
                                     `(dimensions-3d
                                       ,(vector
                                         (sub-value 'width value)
                                         (sub-value 'height value)
                                         (sub-value 'depth value))))
                                    (t resolution-property))))
                          value)))
                     ((eql key 'segment)
                      `(,key
                        ,(mapcar (lambda (segment-property)
                                   (destructuring-bind
                                       (key value) segment-property
                                     (cond ((eql key 'dimensions-2d)
                                            `(dimensions-2d
                                              ,(vector
                                                (sub-value 'width value)
                                                (sub-value 'height value))))
                                           (t segment-property))))
                                 value)))
                     ((eql key 'pose)
                      `(at ,(make-designator
                             'location
                             `((pose ,value)))))
                     ((eql key 'desig-props::detection)
                      (let* ((type-string (second (find 'type value :key #'car))))
                        `(,(intern "TYPE" 'desig-props)
                          ,(intern (string-upcase type-string) 'desig-props))))
                     (t `(,key ,value))))))
    (cpl:mapcar-clean
     (lambda (result)
       (let* ((resolution (cadr (assoc 'resolution
                                       (description result)))))
         (when resolution
           (let ((lastseen (cadr (assoc 'lastseen resolution))))
             (when (and lastseen (<= lastseen 2.0))
               (make-designator
                'object
                (cpl:mapcar-clean #'process-object
                                  (description result))))))))
     results)))

(defun filter-results-on-type (request-desig response-desigs)
  (alexandria:when-let ((request-type (desig-prop-value request-desig 'type)))
    (remove-if-not 
     (lambda (type) (eql request-type type)) response-desigs
     :key (lambda (desig) (desig-prop-value desig 'desig-props:type)))))

(defun call-perception-function (object-designator)
  (let ((results
          (uima:get-uima-result
           (make-designator
            'action `((desig-props::object ,object-designator))))))
    (unless results
      (cpl:fail 'cram-plan-failures:object-not-found
                :object-desig object-designator))
    (filter-results-on-type object-designator (post-process-results results))))

(def-action-handler perceive (object-designator)
  (ros-info (perception) "Perceiving object.")
  (call-perception-function object-designator))

(def-process-module robosherlock-mini-process-module (desig)
  (apply #'call-action (reference desig)))

(def-fact-group robosherlock-mini-action-designators (action-desig)

  (<- (action-desig ?designator (perceive ?object-desig))
    (desig-prop ?designator (to perceive))
    (desig-prop ?designator (obj ?object-desig))
    (current-designator ?object-desig ?current-object-desig)
    (obj-desig? ?current-object-desig)))

(def-fact-group robosherlock-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator robosherlock-mini-process-module)
    (or (desig-prop ?designator (to perceive))))

  (<- (available-process-module robosherlock-mini-process-module)
    (crs:true)))
