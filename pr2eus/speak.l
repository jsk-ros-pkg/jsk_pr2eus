(ros::roseus-add-msgs "sound_play")

(defun speak-jp (jp-str &key google)
  (if google (speak jp-str :lang :ja)
    (progn
      (unless (ros::get-topic-publisher "robotsound_jp")
        (ros::advertise "robotsound_jp" sound_play::SoundRequest 5)
        (unix:sleep 1))
      (ros::publish "robotsound_jp"
                    (instance sound_play::SoundRequest :init
                              :sound sound_play::SoundRequest::*say*
                              :command sound_play::SoundRequest::*play_once*
                              :arg jp-str
                              :arg2 "aq_rm.phont")))))
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

(defun speak (str &key (lang :en) wait)
  (unless (ros::get-topic-publisher "robotsound")
    (ros::advertise "robotsound" sound_play::SoundRequest 5)
    (unix:sleep 1))
  (let* ((query-string
                (escaped-url-string-from-namestring
                  (concatenate string
                               "http://translate.google.com/translate_tts?tl="
                               (string-downcase (string lang))
                               "&ie=UTF-8&q=" str)))
         (sound-msg
          (instance sound_play::SoundRequest :init
                    :sound sound_play::SoundRequest::*play_file*
                    :command sound_play::SoundRequest::*play_once*
                    :arg query-string))
         sound-req-action)
    (ros::ros-debug "sent query: ~A" query-string)
    (if wait
        (progn
          (setq sound-req-action
                (instance ros::simple-action-client :init
                          "/sound_play" sound_play::SoundRequestAction
                          :groupname "speak"))
          (when (not (send sound-request-action :wait-for-server 3))
            (ros::ros-warn "No sound_play action found")
            (return-from speak nil))
          (send sound-req-action :send-goal
                (instance sound_play::SoundRequestActionGoal :init
                          :goal :sound_request sound-msg))
          (send sound-request-action :wait-for-result)
          (send sound-request-action :get-result))
      (ros::publish "robotsound" sound-msg))))

(defun speak-en (en-str &key (google nil) (wait nil))
  (if google (speak en-str :wait wait)
    (progn
      (unless (ros::get-topic-publisher "robotsound")
        (ros::advertise "robotsound" sound_play::SoundRequest 5)
        (unix:sleep 1))
      (let ((sound-msg
             (instance sound_play::SoundRequest :init
                       :sound sound_play::SoundRequest::*say*
                       :command sound_play::SoundRequest::*play_once*
                       :arg en-str)))
        (if wait
            (let ((sound-request-action
                   (instance ros::simple-action-client :init
                             "/sound_play" sound_play::SoundRequestAction
                             :groupname "speak"))
                  goal)
              (if (send sound-request-action :wait-for-server 3)
                  (progn
                    (setq goal (instance sound_play::SoundRequestActionGoal :init))
                    (send goal :goal :sound_request sound-msg)
                    (send sound-request-action :send-goal goal)
                    (send sound-request-action :wait-for-result)
                    (send sound-request-action :get-result))
                (ros::ros-warn "No sound_play action found")))
          (ros::publish "robotsound" sound-msg))))))
