;; speak-test.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")

(load "package://pr2eus/speak.l")
(ros::roseus "test_speak")

(init-unit-test)

(deftest test-speak-en ()
  (assert (speak-en "hello, world" :timeout 10))
;;  (assert (speak-en "hello, world" :timeout 10 :wait t))
  (assert (speak-en "hello, world" :timeout 10 :google t)))

(deftest test-speak-jp ()
  (assert (speak-jp "こんにちは" :timeout 10))
;;  (assert (speak-jp "こんにちは" :timeout 10 :wait t))
  (assert (speak-jp "こんにちは" :timeout 10 :google t)))

(deftest test-speak-google ()
  (assert (speak-google "bonjour" :timeout 10 :lang :fr)))

(run-all-tests)
(exit)

