;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defparameter *speak-wait* nil)

(defun send-speak-msg (msg &key (topic-name "robotsound") (timeout 0) (wait *speak-wait*))
  (if (boundp 'sound_play::soundrequestaction)
      (let ((ac (instance ros::simple-action-client :init
                          topic-name sound_play::SoundRequestAction :groupname "speak"))
            (goal (instance sound_play::SoundRequestActionGoal :init)))
        (unless (send ac :wait-for-server timeout)
          (ros::ros-error "action server /~A is not found. sound_play node is not alive?" topic-name)
          (return-from send-speak-msg nil))
        (send goal :goal :sound_request msg)
        (send ac :send-goal goal)
        (if wait
            (send ac :wait-for-result :timeout timeout) t))
    ;; for backward compatibility
    (progn
      (unless (ros::get-topic-publisher topic-name)
        (ros::advertise topic-name sound_play::SoundRequest 5)
        (unix:sleep 1))
      (ros::publish topic-name msg)
      t)))

(defun speak-google (str &key (lang :ja) (wait *speak-wait*) (topic-name "robotsound") (timeout 20))
  (let* ((qstr (escaped-url-string-from-namestring
                (concatenate string
                             "http://translate.google.com/translate_tts?tl="
                             (string-downcase (string lang))
                             "&client=t&ie=UTF-8&q=" str)))
         (msg (instance sound_play::SoundRequest :init
                        :sound sound_play::SoundRequest::*play_file*
                        :command sound_play::SoundRequest::*play_once*
                        :arg qstr)))
    (send-speak-msg msg
                    :topic-name topic-name
                    :wait wait
                    :timeout timeout)))

(defun speak-jp (str &key google (wait *speak-wait*) (topic-name "robotsound_jp") (timeout 20))
  (when google
      (return-from speak-jp
        (speak-google str :lang :ja :wait wait :timeout timeout)))
  (send-speak-msg
   (instance sound_play::SoundRequest :init
             :sound sound_play::SoundRequest::*say*
             :command sound_play::SoundRequest::*play_once*
             :arg str
             :arg2 "aq_rm.phont")
   :topic-name topic-name
   :wait wait
   :timeout timeout))

(defun speak-en (str &key google (wait *speak-wait*) (topic-name "robotsound") (timeout 20))
  (when google
    (return-from speak-en
            (speak-google str :lang :en :wait wait :timeout timeout)))
  (send-speak-msg
   (instance sound_play::SoundRequest :init
             :sound sound_play::SoundRequest::*say*
             :command sound_play::SoundRequest::*play_once*
             :arg str)
   :topic-name topic-name
   :wait wait
   :timeout timeout))

(provide :speak) ;; end of speak.l
