(load "unittest.l")
(init-unit-test)
(load "package://pr2eus/pr2.l")
(pr2)

(if (and x::*display* (> x::*display* 0)) (objects (list *pr2*)))

(defun test-ik (arm target-coords &rest args)
  (send *pr2* :reset-pose)
  (assert
   (send* *pr2* arm :inverse-kinematics target-coords :debug-view (and x::*display* (> x::*display* 0)) args)
   (format nil "failed to solve (send* *pr2* ~A :inverse-kinematics ~A '~A)~%"
   arm  (list 'make-coords :pos (send target-coords :pos) :rot (send target-coords :rot)) args))
   )

(deftest pr2-test-angle-vector ()
  (send *pr2* :angle-vector))

(deftest pr2-test-ik-1 ()
  (test-ik :larm (make-coords :pos #f(700 0 900)) :rotation-axis t)
  (test-ik :rarm (make-coords :pos #f(700 0 900)) :rotation-axis t))

(run-all-tests)
(exit)
