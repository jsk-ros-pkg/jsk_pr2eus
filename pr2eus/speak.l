;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defparameter *sound-play-clients* (make-hash-table :test #'equal))

(defun play-sound (sound &key arg2 (topic-name "robotsound") wait)
  "Plays sound using sound_play server
   Args:
     sound: if sound is pathname, plays sound file located at given path
            if it is number, server plays builtin sound
            otherwise server plays sound as speech sentence
     topic-name: namespace of sound_play server
     wait: wait until sound is played"
  (let ((msg (instance sound_play::SoundRequest :init
                       :command sound_play::SoundRequest::*play_once*)))
    (cond
      ((numberp sound)
       (send msg :sound sound))
      ((pathnamep sound)
       (send msg :sound sound_play::SoundRequest::*play_file*)
       (send msg :arg (namestring sound)))
      (t
       (send msg :sound sound_play::SoundRequest::*say*)
       (send msg :arg (string sound))))
    (if arg2 (send msg :arg2 arg2))

    (when (boundp 'sound_play::SoundRequestAction)
      (let ((goal (instance sound_play::SoundRequestActionGoal :init))
            (ac (or (gethash topic-name *sound-play-clients*)
                    (instance ros::simple-action-client :init
                              topic-name sound_play::SoundRequestAction
                              :groupname "sound_play"))))
        (when (send ac :wait-for-server 1)
          (when (eq (send ac :get-state) actionlib_msgs::GoalStatus::*active*)
            (send ac :cancel-goal)
            (send ac :wait-for-result :timeout 10))
          (send goal :goal :sound_request msg)
          (setf (gethash topic-name *sound-play-clients*) ac)
          (send ac :send-goal goal)
          (if wait
              (return-from play-sound (send ac :wait-for-result :timeout 10))
              (return-from play-sound t)))))
    ;; use publisher
    (ros::ros-warn "action server /~A not found." topic-name)
    (unless (ros::get-topic-publisher topic-name)
      (ros::advertise topic-name sound_play::SoundRequest 5)
      (unix:sleep 1))
    (ros::publish topic-name msg)
    t))

(defun speak (text &key (lang "") (topic-name "robotsound") wait)
  "Speak sentence using text-to-speech services.
   Args:
     text: sentence to speak
     lang: language to speak, currently :en or :ja are supported.
     topic-name: namespace of sound_play node
     wait: wait the end of speech if enabled"
  (play-sound text
              :topic-name topic-name
              :wait wait
              :arg2 (if (keywordp lang)
                        (string-downcase lang) lang)))

(defun speak-en (text &key (topic-name "robotsound") wait)
  "Speak english sentence"
  (speak text :topic-name topic-name :wait wait))

(defun speak-jp (text &key (topic-name "robotsound_jp") wait)
  "Speak japanese sentence"
  (speak text :lang :ja :topic-name topic-name :wait wait))

(provide :speak) ;; end of speak.l
