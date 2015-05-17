(ros::roseus-add-msgs "sound_play")

(defun speak-jp (jp-str)
  (unless (ros::get-topic-publisher "robotsound_jp")
    (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
    (unix:sleep 1))
  (ros::publish "robotsound_jp"
		(instance sound_play::SoundRequest :init
			  :sound sound_play::SoundRequest::*say*
			  :command sound_play::SoundRequest::*play_once*
			  :arg jp-str
			  :arg2 "aq_rm.phont")))
#| voice text's command
<vtml_accent level=\"5\">text </vtml_ accent> ;;  0 - 5(default) - 9
<vtml_break level=\"1\"/> ;; 0,1,2,3
<vtml_pause time=\"100\"/> ;; msec
<vtml_pitch value=\"100\">text </vtml_pitch> ;; 50-200%
<vtml_sentend level=\"5\"/> ;; 0 - 5(default) - 9
<vtml_speed value=\"100\">text</vtml_speed> ;; 50-400%
<vtml_volume value=\"100\">text</vtml_volume> ;; 0-500%
^^ ;;数字の棒読み
|#
(defun speak-en (en-str &key (google nil) (wait nil))
  (unless (ros::get-topic-publisher "robotsound")
    (ros::advertise "robotsound" sound_play::SoundRequest 5)
    (unix:sleep 1))
  (let (sound-msg)
    (if google
        (setq sound-msg
              (instance sound_play::SoundRequest :init
                        :sound sound_play::SoundRequest::*play_file*
                        :command sound_play::SoundRequest::*play_once*
                        :arg (concatenate string "http://translate.google.com/translate_tts?tl=en&q=" en-str)
                        ))
      (setq sound-msg
            (instance sound_play::SoundRequest :init
                      :sound sound_play::SoundRequest::*say*
                      :command sound_play::SoundRequest::*play_once*
                      :arg en-str
                      ))
      )
    (if wait
        (let ((sound-request-action (instance ros::simple-action-client :init
                                              "/sound_play" sound_play::SoundRequestAction
                                              :groupname "speak"))
              goal
              )
          (if (send sound-request-action :wait-for-server 3)
              (progn
                (setq goal (instance sound_play::SoundRequestActionGoal :init))
                (send goal :goal :sound_request sound-msg)
                (send sound-request-action :send-goal goal)
                (send sound-request-action :wait-for-result)
                (send sound-request-action :get-result)
                )
            (ros::ros-warn "No sound_play action found")
            )
          )
      (ros::publish "robotsound" sound-msg)
      )
    )
  )