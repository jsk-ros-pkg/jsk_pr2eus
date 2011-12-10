(load "unittest.l")

(load "package://pr2eus/make-pr2-model-file")
(ros::roseus "make-pr2-modle-file")
(make-pr2-model-file :output-directory "/tmp")

(init-unit-test)

(when (not (boundp '*pr2-recent*))
  (load "/tmp/pr2.l")
  (setq *pr2-recent* (instance pr2-robot :init))
  (setq *pr2-cameras-recent* (send *pr2-recent* :cameras)))
(when (not (boundp '*pr2-current*))
  (load "package://pr2eus/pr2.l")
  (setq *pr2-current* (instance pr2-robot :init))
  (setq *pr2-cameras-current* (send *pr2-current* :cameras)))

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
  (assert (= (length *pr2-cameras-recent*)
	     (length *pr2-cameras-current*))
	  "check camera number")
  (dolist (camera-current *pr2-cameras-current*)
    (let* ((camera-name (send camera-current :name))
	   (camera-recent (find camera-name *pr2-cameras-recent* :key #'(lambda (x) (send x :name)) :test #'string=)))
      (assert camera-recent
	      (format nil "check camera ~A" camera-name))
      (when camera-recent
	(print (list 'check2 camera-recent))
	(assert (= (norm (send camera-recent :difference-position
			     camera-current)) 0.0)
		(format nil "check camera position for ~A" camera-name))
	(assert (= (norm (send camera-recent :difference-rotation
			       camera-current)) 0.0)
		(format nil "check camera rotation for ~A" camera-name))
	(assert (= (send camera-recent :width)
		   (send camera-current :width))
		(format nil "check camera width for ~A" camera-name))
	(assert (= (send camera-recent :height)
		   (send camera-current :height))
		(format nil "check camera height for ~A" camera-name))
	(assert (= (send camera-recent :fovy)
		   (send camera-current :fovy))
		(format nil "check camera fovy for ~A" camera-name))
	(assert (= (norm (v- (send camera-recent :view-direction)
			     (send camera-current :view-direction))) 0.0)
		(format nil "check camera view-direction for ~A" camera-name))
	(assert (= (norm
		    (array-entity
		     (m- (send camera-recent :projection)
			 (send camera-current :projection)))) 0.0)
		(format nil "check camera projection for ~A" camera-name))
	)))
  )

(run-all-tests)

(exit)
