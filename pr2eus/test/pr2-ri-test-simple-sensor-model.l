(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(pr2-init)

;; pr2 / pr2-sensor-robot test
(deftest instantiate-pr2-sensor-robot-test
  (let (rgb-camera-pr1012 rgb-camera-pr1040)
    ;; call from function
    (pr2)
    (assert *pr2*) ;; generate *pr2*
    (assert (derivedp *pr2* pr2-sensor-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (equal (*pr2* . name) :pr1012)) ;; but we can know serial number
    (setq rgb-camera-1012 (send (send *pr2* :camera :kinect_head/rgb) :viewing :projection))

    (pr2 :pr1040)
    (assert *pr2*) ;; generated *pr2*
    (assert (derivedp *pr2* pr2-sensor-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (equal (*pr2* . name) :pr1040)) ;; but we can know serial number
    (setq rgb-camera-1040 (send (send *pr2* :camera :kinect_head/rgb) :viewing :projection))
    (defun m= (m1 m2) (v= (array-entity m1) (array-entity m2)))
    (when (m= rgb-camera-1012 rgb-camera-1040)
      (warning-message 1 "each robot should have different camera param~%~A~%~A~%"
                       rgb-camera-1040 rgb-camera-1040))

    ;; use instance
    (setq *pr2* (instance pr2-robot :init))
    (assert *pr2*) ;; generate *pr2*
    (assert (derivedp *pr2* pr2-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (null (assoc 'name (send *pr2* :slots)))) ;; but it does not have slot 'name

    (setq *pr2* (instance pr2-sensor-robot :init))
    (assert *pr2*) ;; generate *pr2*
    (assert (derivedp *pr2* pr2-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (equal (*pr2* . name) :pr1012)) ;; but we can know serial number

    (setq *pr2* (instance pr2-sensor-robot :init :pr1012))
    (assert *pr2*) ;; generated *pr2*
    (assert (derivedp *pr2* pr2-sensor-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (equal (*pr2* . name) :pr1012)) ;; but we can know serial number

    (setq *pr2* (instance pr2-sensor-robot :init :pr1040))
    (assert *pr2*) ;; generated *pr2*
    (assert (derivedp *pr2* pr2-sensor-robot)) ;; pr2 derived from pr2-sensor-robot
    (assert (equal (send *pr2* :name) "pr2")) ;; robot name is pr2
    (assert (equal (*pr2* . name) :pr1040)) ;; but we can know serial number
    ))

(run-all-tests)
(exit)
