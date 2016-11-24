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

(deftest test-speak-en-jp ()
  (labels ((test-speak
            ()
            (and (speak '(("en" . "hello, world") ("jp" . "こんにちは")) :timeout 10)
            ;;   (speak '(("en" . "hello, world") ("jp" . "こんにちは")) :wait t :debug t)
                 (speak '(("en" . "hello, world") ("jp" . "こんにちは")) :timeout 10 :google t)
                 (speak '(("en" . "hello, world")) :timeout 10)
                 (speak "hello, world" :timeout 10)
                 )))
    (assert (progn (test-speak)))
    (assert (progn (ros::set-param "speak_language" "jp") (test-speak)))
    (assert (progn (ros::set-param "speak_language" "en") (test-speak)))
    ))

(run-all-tests)
(exit)

