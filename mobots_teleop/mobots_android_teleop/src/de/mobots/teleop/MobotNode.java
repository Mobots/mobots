package de.mobots.teleop;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import path_planner.KeyboardRequestRequest;
import path_planner.KeyboardRequestResponse;
import android.graphics.BitmapFactory;
import android.util.Log;
import android.widget.ImageView;
import android.widget.Toast;

public class MobotNode extends AbstractNodeMain{
	
	public static final Object syncObject = new Object();
	public static double x = 0;
	public static double y = 0;
	public static double theta = 0;
	public static double speed = 1;
	public static double turnSpeed = 1;
	public static boolean keyUp = true;
	
	private ImageView imageView;
	private ServiceClient<KeyboardRequestRequest, KeyboardRequestResponse> client;
	private Subscriber<mobots_msgs.ImageWithPoseAndID> sub;
	
	private String mobot;
	private int id;
	/*private final CancellableLoop loop = new CancellableLoop() {
        protected void loop() throws InterruptedException{
      	  synchronized(MobotNode.syncObject){
      		  syncObject.wait();
      	  }
			twist.setX(x*speed);
			twist.setY(y*speed);
			twist.setTheta(-theta*turnSpeed);
			  publisher.publish(twist);
			  Thread.sleep(300);
        }
      };*/
	
    public void shutdown(){
    	Log.e("mobot", "shutdown");
    	if(client != null){
			KeyboardRequestRequest request = client.newMessage();
			request.setEnable((byte)0);
			request.setMobotId((byte) id);		
			
			client.call(request, new ServiceResponseListener<KeyboardRequestResponse>(){
				public void onFailure(RemoteException arg0) {	
			        Toast.makeText(imageView.getContext(), "failure while releasing keyboard control for mobot: "+mobot, 1).show();
				}
				public void onSuccess(KeyboardRequestResponse res) {}
			});
    	}
    	sub.shutdown();
    	MainActivity.publisher.shutdown();
    }
      
	public MobotNode(String mobotName, int id, ImageView imageView){
		mobot = mobotName;
		this.imageView = imageView;
		this.id = id;
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return GraphName.of("mobot_android_teleop_"+id);
		//return new GraphName("mobot_sixaxis_teleop");
	}
	
	@Override
	public void onStart(final ConnectedNode node){
	    MainActivity.publisher = node.newPublisher("/"+mobot+"/velocity", mobots_msgs.Twist2D._TYPE);
	    sub = node.newSubscriber("/"+mobot+"/image_pose_id", mobots_msgs.ImageWithPoseAndID._TYPE);
	    sub.addMessageListener(new MessageListener<mobots_msgs.ImageWithPoseAndID>(){
			@Override
			public void onNewMessage(final mobots_msgs.ImageWithPoseAndID msg) {
				imageView.post(new Runnable(){
					public void run(){
					    ChannelBuffer buffer = msg.getImage().getData();
					    byte[] data = buffer.array();
					    imageView.setImageBitmap(BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes()));
					}
				});

			}
	    });
	     MainActivity.twist = node.getTopicMessageFactory().newFromType(mobots_msgs.Twist2D._TYPE);
	    try {
			client = node.newServiceClient("/path_planner/keyboard_request", path_planner.KeyboardRequest._TYPE);
			KeyboardRequestRequest request = client.newMessage();
			request.setEnable((byte) 1);
			request.setMobotId((byte) id);
			Log.e("mobotid", "->"+id);
			
			client.call(request, new ServiceResponseListener<KeyboardRequestResponse>(){
				public void onFailure(RemoteException arg0) {	
			        //node.executeCancellableLoop(loop);
				}
				public void onSuccess(KeyboardRequestResponse res) {
					if(res.getEnabled() == 0){
						imageView.setImageResource(android.R.drawable.ic_dialog_alert);
						
						return;
					}
			        //node.executeCancellableLoop(loop);
				}
			});
			
		} catch (ServiceNotFoundException e) {
			Log.e("service not found", e.toString());
	        //node.executeCancellableLoop(loop);
		}
	}	
	
	@Override
	public void onShutdown(Node node){
		shutdown();
	}
}
