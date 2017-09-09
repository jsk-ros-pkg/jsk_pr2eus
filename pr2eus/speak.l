;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defparameter *speak-wait* nil)
(defparameter *speak-action-clients* (make-hash-table))
(defparameter *speak-timeout* 20)

(defun send-speak-msg (msg
                       &key (topic-name "robotsound")
                            (timeout *speak-timeout*)
                            (wait *speak-wait*))
  (cond
    ((boundp 'sound_play::soundrequestaction)
      (let ((goal (instance sound_play::SoundRequestActionGoal :init))
            (action-client-key (intern (string-upcase topic-name) *keyword-package*)))
        (unless (gethash action-client-key *speak-action-clients*)
          (setf (gethash action-client-key *speak-action-clients*)
                (instance ros::simple-action-client :init
                          topic-name sound_play::SoundRequestAction :groupname "speak")))
        (let ((ac (gethash action-client-key *speak-action-clients*)))
          (unless (send ac :wait-for-server timeout)
            (ros::ros-error "action server /~A is not found. sound_play node is not alive?" topic-name)
            (return-from send-speak-msg nil))
          (send goal :goal :sound_request msg)
          (send ac :send-goal goal)
          (if wait
            (send ac :wait-for-result :timeout timeout) t))))
    (t ;; action client is not used for backward compatibility
      (unless (ros::get-topic-publisher topic-name)
        (ros::advertise topic-name sound_play::SoundRequest 5)
        (unix:sleep 1))
      (ros::publish topic-name msg)
      t)))

(defun speak (text &key (lang "") (topic-name "robotsound") wait timeout)
  "Speak sentence using text-to-speech service.

   Args:
    - text: sentence to speak
    - lang: language of sentence
    - topic-name: topic name space for sound_play server
    - wait: wait the end of speech if enabled
    - timeout: timeout for waiting, this is valid only if wait is enabled"
  (send-speak-msg
   (instance sound_play::SoundRequest :init
             :sound sound_play::SoundRequest::*say*
             :command sound_play::SoundRequest::*play_once*
             :arg text
             :arg2 (string-downcase lang))
   :topic-name topic-name
   :wait (or wait *speak-wait*)
   :timeout (or timeout *speak-timeout*)))

(defun speak-jp (text &key wait timeout (topic-name "robotsound_jp"))
  "Speak japanese sentence using text-to-speech service.

   Args:
    - text: sentence to speak
    - topic-name: topic name space for sound_play server
    - wait: wait the end of speech if enabled
    - timeout: timeout for waiting, this is valid only if wait is enabled"
  (speak text :lang "ja" :wait wait :timeout timeout :topic-name topic-name))

(defun speak-en (text &key wait timeout (topic-name "robotsound"))
  "Speak english sentence using text-to-speech service.

   Args:
    - text: sentence to speak
    - topic-name: topic name space for sound_play server
    - wait: wait the end of speech if enabled
    - timeout: timeout for waiting, this is valid only if wait is enabled"
  (speak text :lang "" :wait wait :timeout timeout :topic-name topic-name))

(provide :speak) ;; end of speak.l
