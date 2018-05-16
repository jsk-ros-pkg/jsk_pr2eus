;; test-pr2-tabletop.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "pr2_tabletop_test_node")
(ros::rate 10)

(require :unittest "lib/llib/unittest.l")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(init-unit-test)

(deftest test-pr2-tabletop ()
  (setq *tfl* (instance ros::transform-listener :init))
  (pr2-init)
  (pr2-reset-pose)

  (send *pr2* :head :look-at #f(700 0 700))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq start-time (ros::time-now))
  (setq foundp nil)
  (while (and (not foundp)
              (> (send (ros::time- (ros::time-now) start-time) :to-sec) 120))
    (when (find-if #'(lambda (f)
                       (substringp "segmentation_decomposeroutput" f))
                   (send *tfl* :get-frame-strings))
        (setq foundp t))
    (ros::spin-once)
    (ros::sleep))
  (assert foundp "tabletop object not found"))

(unix:sleep 20)
(run-all-tests)
(exit)
