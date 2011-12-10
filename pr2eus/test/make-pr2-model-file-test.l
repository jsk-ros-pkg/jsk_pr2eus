(load "unittest.l")

;(load "package://pr2eus/make-pr2-model-file")
;(ros::roseus "make-pr2-modle-file")
;(make-pr2-model-file :output-directory "/tmp")

(init-unit-test)

(when (not (boundp '*pr2-recent*))
  (load "/tmp/pr2.l")
  (setq *pr2-recent* (instance pr2-robot :init)))
(when (not (boundp '*pr2-current*))
  (load "package://pr2eus/pr2.l")
  (setq *pr2-current* (instance pr2-robot :init)))

(assert (= (norm (v- (send *pr2-recent* :angle-vector)
		     (send *pr2-current* :angle-vector))) 0.0)
	"check default angle-vector")
(assert (= (norm (v- (send *pr2-recent* :reset-pose)
		     (send *pr2-current* :reset-pose))) 0.0)
	"check reset-pose angle-vector")
(send *pr2-recent* :angle-vector (send *pr2-current* :angle-vector))

(deftest test-link ()
  (assert (=
	   (length (send *pr2-recent* :links))
	   (length (send *pr2-current* :links)))
	  "check number of link"
	  )
  (mapcar #'(lambda (link-recent link-current)
	      (assert (eq (send link-recent :name) (send link-current :name))
		      "check link name")
	      (assert
	       (= (norm (send (send link-recent :worldcoords) :difference-position (send link-current :worldcoords))) 0.0)
	       "check link position")
	      (assert
	       (= (norm (send (send link-recent :worldcoords) :difference-rotation (send link-current :worldcoords))) 0.0)
	       "check link orientation"))
	  (send *pr2-recent* :links) (send *pr2-current* :links))
  )

(deftest test-joint
  (assert (=
	   (length (send *pr2-recent* :joint-list))
	   (length (send *pr2-current* :joint-list)))
	  "check number of joint")

  (mapcar #'(lambda (joint-recent joint-current)
	      (assert (eq (send joint-recent :name) (send joint-current :name))
		      "check joint name")
	      (assert
	       (= (send joint-recent :min-angle)  (send joint-current :min-angle))
	       (format nil "check min angle between ~A ~A" (send joint-recent :name) (send joint-current :name)))
	      (assert
	       (= (send joint-recent :max-angle)  (send joint-current :max-angle))
	       (format nil "check max angle between ~A ~A" (send joint-recent :name) (send joint-current :name))
	      ))
	  (send *pr2-recent* :joint-list) (send *pr2-current* :joint-list))
  )


(deftest test-end-coords
  (dolist (limb (list :larm :rarm :head))
    (assert (= (norm (send (send *pr2-recent* limb :end-coords) :difference-position
			      (send *pr2-current* limb :end-coords))) 0)
	    (format nil "strict check end-coords position of ~A" limb))
    (assert (= (norm (send (send *pr2-recent* limb :end-coords) :difference-rotation
			      (send *pr2-current* limb :end-coords))) 0)
	    (format nil "strict check end-coords rotation of ~A" limb))
    (assert (eps= (norm (send (send *pr2-recent* limb :end-coords) :difference-position
			      (send *pr2-current* limb :end-coords))) 0)
	    (format nil "check end-coords position of ~A" limb))
    (assert (eps= (norm (send (send *pr2-recent* limb :end-coords) :difference-rotation
			      (send *pr2-current* limb :end-coords))) 0)
	    (format nil "check end-coords rotation of ~A" limb))
    ))

(deftest test-jacobian
  (dolist (limb (list :larm :rarm :head))
    (assert
     (eps=
      (norm
       (array-entity
	(m-
	 (send *pr2-recent* :calc-jacobian-from-link-list
	       (send *pr2-recent* :link-list (send *pr2-recent* limb :end-coords :parent))
	       :move-target (send *pr2-recent* limb :end-coords))
	 (send *pr2-current* :calc-jacobian-from-link-list
	       (send *pr2-current* :link-list (send *pr2-current* limb :end-coords :parent))
	       :move-target (send *pr2-current* limb :end-coords))))
       )
      0.0)
     (format nil "check jacobian for ~A~" limb))))

(deftest test-cameras
  (assert (= (length (send *pr2-recent* :cameras))
	     (length (send *pr2-current* :cameras)))
	  "check camera number")
  (dolist (camera (send-all (send *pr2-current* :cameras) :name))
    (assert (send *pr2-recent* :camera camera)
	    (format nil "check camera ~A" camera))
    (assert (= (norm (send (send *pr2-recent* :camera camera) :difference-position
			   (send *pr2-recent* :camera camera))) 0.0)
	    (format nil "check camera position for ~A" camera))
    (assert (= (norm (send (send *pr2-recent* :camera camera) :difference-rotation
			   (send *pr2-recent* :camera camera))) 0.0)
	    (format nil "check camera rotation for ~A" camera))
    (assert (= (send (send *pr2-recent* :camera camera) :width)
	       (send (send *pr2-recent* :camera camera) :width))
	    (format nil "check camera width for ~A" camera))
    (assert (= (send (send *pr2-recent* :camera camera) :height)
	       (send (send *pr2-recent* :camera camera) :height))
	    (format nil "check camera height for ~A" camera))
    (assert (= (send (send *pr2-recent* :camera camera) :fovy)
	       (send (send *pr2-recent* :camera camera) :fovy))
	    (format nil "check camera fovy for ~A" camera))
    (assert (= (norm (v- (send (send *pr2-recent* :camera camera) :view-direction)
			 (send (send *pr2-recent* :camera camera) :view-direction))) 0.0)
	    (format nil "check camera view-direction for ~A" camera))
    (assert (= (norm
		(array-entity
		 (m- (send (send *pr2-recent* :camera camera) :projection)
		     (send (send *pr2-recent* :camera camera) :projection)))) 0.0)
	    (format nil "check camera projection for ~A" camera))
    ))




(run-all-tests)

(exit)
