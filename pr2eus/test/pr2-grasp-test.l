(require :unittest "lib/llib/unittest.l")
(init-unit-test)
(load "package://pr2eus/pr2-interface.l")
(pr2)

(setq *ri* (instance pr2-interface :init))

(deftest test-start-grasp ()
  (send *ri* :start-grasp)
  )

(deftest test-stop-grasp ()
  (send *ri* :stop-grasp)
  )

(test-start-grasp)
(test-stop-grasp)
(run-all-tests)
(exit)

