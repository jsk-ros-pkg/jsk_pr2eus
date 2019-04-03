#!/usr/bin/env roseus
;; pr2-tabletop-object-grasp.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defvar *grasp-target-bounding-box-topic* "/bounding_box_marker/selected_box")
(defvar *grasp-status-topic* "/tabletop_object_grasp_status")
(defvar *base-frame-id* "/base_footprint")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_rviz_plugins")

(ros::roseus "pr2_tabletop_object_grasp_node")
(ros::rate 10)

(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun pr2-pregrasp-pose ()
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation))

(defun publish-status (msg-type fmt)
  (let ((color
         (case msg-type
          ('error (instance std_msgs::ColorRGBA :init
                            :r 1.0 :g 0.0 :b 0.0 :a 1.0))
          ('warn (instance std_msgs::ColorRGBA :init
                           :r 1.0 :g 1.0 :b 0.0 :a 1.0))
          (t (instance std_msgs::ColorRGBA :init
                       :r (/ 157.0 255) :g (/ 204.0 255) :b (/ 224.0 255) :a 1.0)))))
    (unless (ros::get-topic-publisher *grasp-status-topic*)
      (ros::advertise *grasp-status-topic* jsk_rviz_plugins::OverlayText 5)
      (unix:sleep 1))
    (ros::publish *grasp-status-topic*
                  (instance jsk_rviz_plugins::OverlayText :init
                            :width 1920 :height 400
                            :left 0 :top 0
                            :line_width 10 :text_size 50
                            :fg_color color
                            :text fmt))))

(defun publish-info (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-info s)
    (publish-status 'info s)))
(defun publish-warn (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-warn s)
    (publish-status 'warn s)))
(defun publish-error (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-error s)
    (publish-status 'error s)))

(defun vector3-> (applier v)
  (funcall applier
         (* (send v :x) 1000.0)
         (* (send v :y) 1000.0)
         (* (send v :z) 1000.0)))

(defun bounding-box->cube (msg)
  (let ((cds (ros::tf-pose->coords (send msg :pose)))
        (d (vector3-> 'list (send msg :dimensions)))
        (base->parent (send *tfl* :lookup-transform
                            *base-frame-id* (send msg :header :frame_id)
                            (ros::time 0))))
    (send *tfb* :send-transform
          (send (send base->parent :copy-worldcoords)
                :transform cds)
          *base-frame-id* "target_object" (send msg :header :stamp))
    (send (apply #'make-cube d)
          :transform
          (send (send base->parent :copy-worldcoords)
                :transform cds))))

(defun grasp-target-callback (msg)
  (let ((target-obj (bounding-box->cube msg)))
    (if *grasping-object-p*
        (progn
          (publish-warn "cancel grasping...")
          (send *ri* :stop-motion)
          (send *ri* :stop-grasp *arm* :wait t)
          (setq *grasping-object-p* nil))
      (setq *grasping-object-p* t))
    (unless
      (send *pr2* *arm* :inverse-kinematics
            (make-coords :pos (v+ (send target-obj :worldpos)
                                  (float-vector -120 0 0))
                         :rpy #f(0 0 0))
            :rotation-axis t)
      (publish-error "out of reach...")
      (return-from grasp-target-callback nil))
    (publish-info "openning gripper...")

    (setq *grasp-state* "reaching")
    (publish-grasp-state)
    (publish-info "pre grasp pose...")
    (send *ri* :stop-grasp *arm* :wait t)
    (send *pr2* :head :look-at-hand *arm*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
    (send *ri* :wait-interpolation)

    (setq *grasp-state* "grasp-approaching")
    (publish-grasp-state)
    (publish-info "reaching...")
    (send *pr2* *arm* :move-end-pos #f(160 0 0) :world)
    (send *pr2* :head :look-at-hand *arm*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (setq *grasp-state* "grasp-closing")
    (publish-grasp-state)
    (publish-info "grasping...")
    (let ((grasp-result (send *ri* :start-grasp *arm*)))
      (send *ri* :wait-interpolation)
      (unless (< 5.0 grasp-result)
        (publish-error "failed to grasp ~A" grasp-result)
        (send *ri* :stop-grasp *arm* :wait t)
        (publish-info "back to pre grasp...")
        (pr2-pregrasp-pose)
        (return-from grasp-target-callback nil)))

    (setq *grasp-state* "picking")
    (publish-grasp-state)
    (publish-info "picking up...")
    (send *pr2* *arm* :move-end-pos #f(0 0 150) :world)
    (send *pr2* :head :look-at-hand *arm*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (publish-info "grasp succeeded!")

    (setq *grasp-state* "placing")
    (publish-grasp-state)
    (publish-info "placing ...")
    (send *pr2* *arm* :move-end-pos #f(0 0 -150) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    (setq *grasp-state* "grasp-opening")
    (publish-grasp-state)
    (send *ri* :stop-grasp *arm* :wait t)
    (publish-info "return object...")

    (setq *grasp-state* "leaving")
    (publish-grasp-state)
    (send *pr2* *arm* :move-end-pos #f(-160 0 0) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (publish-info "pre grasp pose")

    (setq *grasp-state* "returning")
    (publish-grasp-state)
    (pr2-pregrasp-pose)
    (publish-info "click tabletop object!")
    ))


(defun wait-for-grasp-target ()
  (ros::subscribe *grasp-target-bounding-box-topic*
                  jsk_recognition_msgs::BoundingBox #'grasp-target-callback)
  (publish-info "click tabletop object!"))

(defun publish-grasp-state ()
  (ros::publish "/grasp_state" (instance std_msgs::String :init :data *grasp-state*)))

(defun demo ()
  (setq *grasping-object-p* nil)
  (setq *arm* :rarm)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (ros::advertise "/grasp_state" std_msgs::String 1)
  (setq *grasp-state* "waiting")
  (pr2-init)
  (pr2-pregrasp-pose)
  (wait-for-grasp-target)
  (ros::spin))
(demo)
