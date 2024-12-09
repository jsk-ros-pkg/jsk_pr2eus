;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defparameter *sound-play-clients* (make-hash-table :test #'equal))

(defun play-sound (sound &key arg2 (topic-name "robotsound") wait (volume 1.0))
  "Plays sound using sound_play server
   Args:
     sound: if sound is pathname, plays sound file located at given path
            if it is number, server plays builtin sound
            otherwise server plays sound as speech sentence
     topic-name: namespace of sound_play server
     wait: wait until sound is played
     volume: Volume at which to play the sound. 0 is mute, 1.0 is 100%."
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
    ;; since sound_play version 0.3.1, the message
    ;; including volume slot. c.f.
    ;; https://github.com/ros-drivers/audio_common/pull/51
    (when (find-method msg :volume)
      (send msg :volume volume))

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
          (send goal :header :stamp (ros::time-now))
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

(defun speak (text &key (lang "") (topic-name "robotsound") wait (volume 1.0))
  "Speak sentence using text-to-speech services.
   Args:
     text: sentence to speak
     lang: language to speak, currently :en or :ja are supported.
     topic-name: namespace of sound_play node
     wait: wait the end of speech if enabled
     volume: Volume at which to play the sound. 0 is mute, 1.0 is 100%."
  (play-sound text
              :topic-name topic-name
              :wait wait
              :volume volume
              :arg2 (if (keywordp lang)
                        (string-downcase lang) lang)))

(defun speak-en (text &key (topic-name "robotsound") wait (volume 1.0))
  "Speak english sentence"
  (speak text :topic-name topic-name :wait wait :volume volume))

(defun speak-jp (text &key (topic-name "robotsound_jp") wait (volume 1.0))
  "Speak japanese sentence"
  (speak text :lang :ja :topic-name topic-name :wait wait :volume volume))

(defun speak
  (speach-string-candidates
   &rest args)
  "Speak with language selection.
   Argument:
     speach-string-candidates should be two types:
       List of cons of language name and speak-string. For example, '( (\"en\" . \"Hello\") (\"jp\" . \"こんにちわ\") )
       English speak-string. For example, \"Hello\", which is same as '( (\"en\" . \"Hello\") ).
     If adequate argument are not specified, english string are input to speak-google.
   Speak language name:
     Speak language name is switched by 'speak_language' rosparam.
     If no such rosparam is specified, speak \"en\".
     If users do not use speak-google, speak_language should be related with speak-xx functinos, such as speak-jp and speak-en. Otherwise, speak function invokes error."
  ;; Convert atom argument to candidate list
  (if (stringp speach-string-candidates)
      (setq speach-string-candidates (list (cons "en" speach-string-candidates))))
  ;; Check speak_language
  (let* ((lang (or (ros::get-param "speak_language") "en"))
         (speak-string (cdr (assoc lang speach-string-candidates :test #'string=)))
         (speak-func
          (when (functionp (intern (read-from-string (format nil "'speak-~A" lang))))
            (intern (read-from-string (format nil "#'speak-~A" lang))))))
    ;; Check argument
    (unless speak-string
      (warn ";; No speach string are specified for ~A from argumenta!! Use english and speak-google!!~%" lang)
      (setq speak-string (cdr (assoc "en" speach-string-candidates :test #'string=)))
      (setq speak-func #'speak-google))
    ;; Check speak-xx function existence.
    (unless speak-func
      (warn ";; No such speak function (speak-~A) defined!! Using google engine~%" lang)
      (setq speak-func #'speak-google))
    (warn ";; ~A~%" (list (cadr speak-func) speak-string args))
    ;; Speak
    (apply speak-func speak-string args)
    ))

(provide :speak) ;; end of speak.l
