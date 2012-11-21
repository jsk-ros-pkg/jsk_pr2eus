#!/usr/bin/env roseus
(ros::roseus "realtime_tabletop")
(load "package://pr2eus_tutorials_obsolated/euslisp/template_grasp_samples.l")

(defun setup ()
  (ros::advertise "image_marker" image_view2::ImageMarker2 10)
  (ros::advertise "tabletop_detection_marker_array" visualization_msgs::MarkerArray 10)
  (ros::set-param "stop_tabletop" 0)
  (setq *rate* 5)
  (ros::rate (/ 1.0 *rate*))
  )

(setup)
(ros::ros-info "waiting service: tabletop_segmentation")
(ros::wait-for-service "/tabletop_segmentation")

(do-until-key
 (ros::spin-once)
 (ros::ros-info "Scanning object")
 (when (zerop (ros::get-param "stop_tabletop"))
   (let ((pc-list (tabletop-detect)))
     (when pc-list
       (visualize-object-image pc-list :outline-colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 0.8 :a 1.0)) :lifetime *rate* :use-text nil :ns "tabletop")
       (visualize-object-rviz pc-list :lifetime *rate* :ns "tabletop")
       )))
 (ros::sleep))
