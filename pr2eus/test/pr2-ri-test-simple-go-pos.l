(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(pr2-init)

;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165#discussion_r37421484
(deftest test-go-pos
  (let ()
    (setq *ri* (instance pr2-interface :init))
    (assert (send *ri* :go-pos 1 0 0) "(send *ri* :go-pos 1 0 0)") ;; go-pos is relative to current position
    (assert (send *ri* :go-pos 0 1 90) "(send *ri* :go-pos 0 1 90)")
    (assert (send *ri* :go-pos-no-wait -1 1 -90) "(send *ri* :go-pos-no-wait -1 1 -90)")
    (ros::sleep 1) ;; wait for 1 sec to activate goal status...
    (assert (send *ri* :go-waitp) "(send *ri* :go-waitp)")
    (assert (send *ri* :go-wait) "(send *ri* :go-wait)")
    (assert (eps-v= (send (send *ri* :worldcoords ) :worldpos) #f(0 0 0)))
    (assert (send *ri* :go-pos-unsafe-no-wait -1 1 -90) "(send *ri* :go-pos-unsafe-no-wait -1 1 -90)")
    ))

(deftest test-move-to
  (let ()
    (setq *ri* (instance pr2-interface :init))
    (assert (send *ri* :move-to (make-coords :pos #f(1000 0 0))) "(send *ri* :move-to (make-coords :pos #f(1000 0 0)))") ;; default is world and wait
    (send *ri* :move-to (make-coords :pos #f(1000 1000 0) :rpy (float-vector pi/2 0 0)))
    (assert (send *ri* :move-to (make-coords) :no-wait t) "(send *ri* :move-to (make-coords) :no-wait t)") ;; no-wait t means not wait so need to call wait
    (assert (send *ri* :move-to-wait) "(send *ri* :move-to-wait)") ;; wait move-to
    (assert (eps-v= (send (send *ri* :worldcoords ) :worldpos) #f(0 0 0)))
    ))

(run-all-tests)
(exit)
