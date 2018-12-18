;; speak.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "sound_play")

(defparameter *speak-wait* nil)
(defparameter *speak-action-clients* (make-hash-table))
(defparameter *speak-timeout* 20)

(defun send-speak-msg (msg
                       &key (topic-name "robotsound") (timeout *speak-timeout*)
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

(defun speak-google (str &key (lang :ja) (wait *speak-wait*) (topic-name "robotsound") (timeout *speak-timeout*))
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

(defun speak-jp (str &key google (wait *speak-wait*) (topic-name "robotsound_jp") (timeout *speak-timeout*))
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

(defun speak-en (str &key google (wait *speak-wait*) (topic-name "robotsound") (timeout *speak-timeout*))
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
          (if (fboundp (eval (read-from-string (format nil "'speak-~A" lang))))
              (eval (read-from-string (format nil "#'speak-~A" lang))))))
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
