(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/robot-interface.l")
(load "package://euslisp/jskeus/irteus/demo/sample-robot-model.l")

(init-unit-test)

(defclass sample-robot-interface
  :super robot-interface
  :slots ())
(defmethod sample-robot-interface
  (:init
   (&rest args)
   (send-super* :init :robot sample-robot args)
   self))

(when (not (boundp '*ri*))
  (setq *robot* (instance sample-robot :init))
  (setq *ri* (instance sample-robot-interface :init)))


(deftest test-angle-vector
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  )

(deftest test-wait-interpolation
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (null (send *ri* :wait-interpolation)))
  )

(deftest test-state
  (assert (null (send *ri* :state)))
  )

(deftest test-potentio-vector
  (assert (send *robot* :reset-pose))
  (assert (send *ri* :angle-vector (send *robot* :angle-vector) 2000))
  (assert (null (send *ri* :wait-interpolation)))
  (assert (< (norm (v- (send *robot* :angle-vector) (send *ri* :potentio-vector))) 10.0))
  )

(deftest test-worldpos
  (assert (send *ri* :worldcoords))
  )


(run-all-tests)
(exit)



