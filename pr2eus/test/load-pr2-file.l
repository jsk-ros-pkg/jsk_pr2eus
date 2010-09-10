#!/usr/bin/env roseus

(load "package://pr2eus/pr2-utils.l")
(roseus "test_load_pr2_file")
(load-pr2-file)

;; check for both
(pr2)
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(objects (list *pr2*))

;; check optical frame
(dolist (frame (list :wide_stereo_optical_frame
                     :narrow_stereo_optical_frame))
  (warn "frame ~A -> ~A~%" frame (send *pr2* frame)))
