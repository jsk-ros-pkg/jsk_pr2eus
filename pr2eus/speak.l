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