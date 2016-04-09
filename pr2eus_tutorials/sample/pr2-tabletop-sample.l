#!/usr/bin/env roseus

;; pr2-tabletop-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "get_objects_node")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(setq *tfl* (instance ros::transform-listener :init))

(pr2-init)
(pr2-reset-pose)
(send *ri* :stop-grasp :rarm :wait t)

;; look at table and objects
(send *pr2* :head :look-at #f(700 0 700))
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

;; get coords from base footprint of PR2 to a detected object
(setq base->obj1 (send *tfl* :lookup-transform "/base_footprint" "/tabletop_object1" (ros::time 0)))

;; pre grasp pose
(send *pr2* :rarm :inverse-kinematics base->obj1 :rotation-axis :z)
(send *pr2* :rarm :move-end-pos #f(-200 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;; reach to object
(send *pr2* :rarm :move-end-pos #f(210 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;; grasp object
(send *ri* :start-grasp :rarm)

;; bring up object
(send *pr2* :rarm :move-end-pos #f(0 0 200))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(provide :pr2-tabletop-sample) ;; end of pr2-tabletop-sample.l
