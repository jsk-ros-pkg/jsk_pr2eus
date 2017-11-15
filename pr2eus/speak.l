;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defun speak-jp (text &key wait timeout (topic-name "robotsound_jp"))
  "Speak japanese sentence using text-to-speech service.

   Args:
    - text: sentence to speak
    - topic-name: topic name space for sound_play server
    - wait: wait the end of speech if enabled
    - timeout: timeout for waiting, this is valid only if wait is enabled"

  (unless (boundp '*ri*)
    (ros::ros-error "Instantiate *ri* first to use text-to-speech.")
    (return-from speak-jp nil))
  (ros::ros-warn "The function `speak-jp` is deprecated, please use (send *ri* :speak-jp \"text\")")

  (let ((timeout-bak (send *ri* :speak-timeout)))
    (when timeout
      (send *ri* :speak-timeout timeout))
    (prog1
        (send *ri* :speak-jp text :wait wait :topic-name topic-name)
      (when timeout
        (send *ri* :speak-timeout timeout-bak)))))

(defun speak-en (text &key wait timeout (topic-name "robotsound"))
  "Speak english sentence using text-to-speech service.

   Args:
    - text: sentence to speak
    - topic-name: topic name space for sound_play server
    - wait: wait the end of speech if enabled
    - timeout: timeout for waiting, this is valid only if wait is enabled"

  (unless (boundp '*ri*)
    (ros::ros-error "Instantiate *ri* first to use text-to-speech.")
    (return-from speak-en nil))
  (ros::ros-warn "The function `speak-en` is deprecated, please use (send *ri* :speak-en \"text\")")

  (let ((timeout-bak (send *ri* :speak-timeout)))
    (when timeout
      (send *ri* :speak-timeout timeout))
    (prog1
        (send *ri* :speak-en text :wait wait :topic-name topic-name)
      (when timeout
        (send *ri* :speak-timeout timeout-bak)))))

(provide :speak) ;; end of speak.l
