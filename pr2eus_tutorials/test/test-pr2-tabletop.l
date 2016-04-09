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

  (unix:sleep 3)

  (setq base->ball (send *tfl* :lookup-transform "/base_footprint" "/tabletop_object1" (ros::time 0)))
  (setq base->beer (send *tfl* :lookup-transform "/base_footprint" "/tabletop_object2" (ros::time 0)))
  (setq base->cylinder (send *tfl* :lookup-transform "/base_footprint" "/tabletop_object3" (ros::time 0)))

  (ros::ros-info "base->ball: ~A" base->ball)
  (ros::ros-info "base->beer: ~A" base->beer)
  (ros::ros-info "base->cylinder: ~A" base->cylinder)

  (assert base->ball "base_footprint -> ball(tabletop_object1) not resolved in tf")
  (assert base->beer "base_footprint -> beer(tabletop_object2) not resolved in tf")
  (assert base->cylinder "base_footprint -> cylinder(tabletop_object3) not resolved in tf"))

(unix:sleep 20)
(run-all-tests)
(exit)
