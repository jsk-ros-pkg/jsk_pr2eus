#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_impedance/pr2eus-impedance.l")

(init-unit-test)

(deftest test-impedance
  ;;
  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

  (send *ri* :start-impedance :rarm)
  (send *ri* :start-impedance-trajectory
        :rarm (list (list (make-coords :pos #f(550 0 950)) 2000.0
                          :control-rot (send (make-coords) :worldrot)
                          :target-force #f(-10 1000 1000 20 20 20)
                          :force-control (list t nil nil nil nil nil))
                    (list (make-coords :pos #f(550 40 950)) 4000.0
                          :control-rot (send (make-coords) :worldrot)
                          :target-force #f(-10 1000 1000 20 20 20)
                          :force-control (list t nil nil nil nil nil))))
  (send *ri* :wait-impedance-trajectory :rarm 10)
  (send *ri* :stop-impedance :rarm)
  )

(run-all-tests)
(exit)

