from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


#This works somewhat now.
def on_message_lost(event, logger):
    logger.info(f"Message lost. Total count={event.total_count}")

def on_deadline_missed(event, logger):
    logger.info(f"Deadline missed. Total count={event.total_count}")


#Reliable, no history on restart, store last 10 messages
qos_revo = QoSProfile(depth=100)
qos_revo.reliability = QoSReliabilityPolicy.RELIABLE
qos_revo.durability = QoSDurabilityPolicy.VOLATILE
qos_revo.history = QoSHistoryPolicy.KEEP_LAST

#Fast, may drop packets, no history on restart, store last 10 messages
qos_bevo = QoSProfile(depth=100)
qos_bevo.reliability = QoSReliabilityPolicy.BEST_EFFORT
qos_bevo.durability = QoSDurabilityPolicy.VOLATILE
qos_bevo.history = QoSHistoryPolicy.KEEP_LAST

#Reliable, keeps last 10 messages for late joiners
qos_retr = QoSProfile(depth=100)
qos_retr.reliability = QoSReliabilityPolicy.RELIABLE
qos_retr.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
qos_retr.history = QoSHistoryPolicy.KEEP_LAST

#Fast, may drop packets, keeps last 10 messages for late joiners
qos_betr = QoSProfile(depth=100)
qos_betr.reliability = QoSReliabilityPolicy.BEST_EFFORT
qos_betr.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
qos_betr.history = QoSHistoryPolicy.KEEP_LAST

#Reliable, keeps messages in history until given to subscribers but not after restart
qos_reka = QoSProfile(depth=1000)
qos_reka.reliability = QoSReliabilityPolicy.RELIABLE
qos_reka.durability = QoSDurabilityPolicy.VOLATILE
qos_reka.history = QoSHistoryPolicy.KEEP_ALL


#Fast, may drop packets, keeps messages in history until given to subscribers but not after restart
qos_beka = QoSProfile(depth=1000)
qos_beka.reliability = QoSReliabilityPolicy.BEST_EFFORT
qos_beka.durability = QoSDurabilityPolicy.VOLATILE
qos_beka.history = QoSHistoryPolicy.KEEP_ALL

QOS_PROFILES = {'reliable_volatile': qos_revo, 'best_effort_volatile' : qos_bevo, 'reliable_transient_local' : qos_retr, 'best_effort_transient_local' :  qos_betr, 'reliable_keep_all' : qos_reka, 'best_effort_keep_all' : qos_beka}