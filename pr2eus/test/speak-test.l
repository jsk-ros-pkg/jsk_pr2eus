;; speak-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(load "irteus/demo/sample-robot-model.l")
(load "package://pr2eus/robot-interface.l")
(load "package://pr2eus/speak.l")

(ros::roseus "test_speak")

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
  (setq *ri* (instance sample-robot-interface :init)))


(deftest test-speak-en ()
  (assert (speak-en "hello, world" :timeout 10))
  (assert (send *ri* :speak-en "hello, world"))
  (assert (send *ri* :speak-en "good bye!" :wait t)))

(deftest test-speak-jp ()
  (assert (speak-jp "こんにちは" :timeout 10))
  (assert (send *ri* :speak-jp "こんにちは"))
  (assert (send *ri* :speak-jp "またね" :wait t)))

(run-all-tests)
(exit)

