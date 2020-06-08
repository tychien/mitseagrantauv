
(cl:in-package :asdf)

(defsystem "dataprocess-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ClusterArray" :depends-on ("_package_ClusterArray"))
    (:file "_package_ClusterArray" :depends-on ("_package"))
  ))