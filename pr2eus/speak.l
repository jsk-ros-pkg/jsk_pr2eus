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
(defun speak-en (en-str)
  (unless (ros::get-topic-publisher "robotsound")
    (ros::advertise "robotsound" sound_play::SoundRequest 5)
    (unix:sleep 1))
  (ros::publish "robotsound"
		(instance sound_play::SoundRequest :init
			  :sound sound_play::SoundRequest::*say*
			  :command sound_play::SoundRequest::*play_once*
			  :arg en-str
			  )))