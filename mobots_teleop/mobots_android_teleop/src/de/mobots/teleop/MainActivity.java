/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package de.mobots.teleop;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.CheckBox;

/**
 * An app that can be used to control a remote robot. This app also demonstrates
 * how to use some of views from the rosjava android library.
 * 
 * @author munjaldesai@google.com (Munjal Desai)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class MainActivity extends RosActivity implements SensorEventListener, OnItemSelectedListener, OnCheckedChangeListener{

	private NodeMainExecutor mainExec;
	private MobotNode currentMobot;
	private NodeConfiguration nodeConfig;
	public static SensorManager sensorManager;
	boolean speedDown;
	public static Publisher<mobots_msgs.Twist2D> publisher;
	public static double x = 0;
	public static double y = 0;
	public static double theta = 0;
	public static double speed = 1;
	public static double turnSpeed = 1;
	public static boolean keyUp = true;
	public static mobots_msgs.Twist2D twist;
	
	public void onCheckedChanged(CompoundButton arg0, boolean arg1) {
		if(arg1)
			sensorManager.registerListener(this,
					sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
					SensorManager.SENSOR_DELAY_UI);
		else
			sensorManager.unregisterListener(this);
	}
	
    public void onItemSelected(AdapterView<?> parent, View view, int pos, long id) {
    	String speeds = (String) parent.getItemAtPosition(pos);
    	if(parent.getId() == R.id.spinner)
    		speed = Double.parseDouble(speeds);
    	else
    		turnSpeed = Double.parseDouble(speeds);
    }

    public void onNothingSelected(AdapterView<?> parent) {}
	public void onAccuracyChanged(Sensor sensor,int accuracy){}

	public void onSensorChanged(SensorEvent event){
			// assign directions
			//float x=event.values[0];
			//float z=event.values[2];
		if(!speedDown)
			return;
		//if(event.values[1] < 0.8)event.values[1] = 0;
		twist.setTheta(event.values[1]/SensorManager.GRAVITY_EARTH*turnSpeed);
		twist.setX(speed);
		publisher.publish(twist);
	}
  
	
	@Override
	public void onStart(){
		super.onStart();
		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_UI);
	}
	
	@Override
	public void onPause(){
		super.onPause();
		sensorManager.unregisterListener(this);
	}

  public MainActivity() {
    super("Teleop", "Teleop");
  }
  
  private final View.OnClickListener setHandler = new View.OnClickListener(){

	@Override
	public void onClick(View v) {
		if(mainExec == null)return;
		if(currentMobot != null){
			currentMobot.shutdown();
			mainExec.shutdownNodeMain(currentMobot);
		}
		EditText t = (EditText) MainActivity.this.findViewById(R.id.edit);
		String mobot = t.getText().toString();
		currentMobot = new MobotNode(mobot, Integer.parseInt(""+mobot.charAt(5)), (ImageView) findViewById(R.id.pic));
		mainExec.execute(currentMobot, nodeConfig);
		findViewById(R.id.status).setVisibility(View.GONE);
	}
  };
  
  private final View.OnTouchListener gasHandler = new View.OnTouchListener() {
	public boolean onTouch(View v, MotionEvent event) {
		switch(event.getAction()){
		case MotionEvent.ACTION_DOWN:
			twist.setTheta(0);
			twist.setX(speed);
			publisher.publish(twist);
			speedDown = true;
			/*synchronized(MobotNode.syncObject){
				MobotNode.syncObject.notify();
			  }*/
			break;
		case MotionEvent.ACTION_UP:
			twist.setTheta(0);
			twist.setX(0);
			publisher.publish(twist);
			speedDown = false;
			break;
		}
		return false;
	}
};

	private final View.OnTouchListener turnHandler = new View.OnTouchListener() {
		public boolean onTouch(View v, MotionEvent event) {
			switch(event.getAction()){
			case MotionEvent.ACTION_DOWN:
				twist.setTheta(1);
				twist.setX(0);
				publisher.publish(twist);
				break;
			case MotionEvent.ACTION_UP:
				twist.setTheta(0);
				twist.setX(0);
				publisher.publish(twist);
				break;
			}
			return false;
		}
	};

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    this.getWindow().getDecorView().setFocusable(false);
    this.getWindow().getDecorView().setFocusableInTouchMode(false);
    findViewById(R.id.go).setOnClickListener(setHandler);
	sensorManager=(SensorManager)getSystemService(SENSOR_SERVICE);
	findViewById(R.id.gas).setOnTouchListener(gasHandler);
	findViewById(R.id.turn).setOnTouchListener(turnHandler);
	CheckBox check = (CheckBox) findViewById(R.id.check);
	check.setOnCheckedChangeListener(this);
	Spinner s = (Spinner) findViewById(R.id.spinner);
	Spinner s2 = (Spinner) findViewById(R.id.spinner2);
	String[] speeds = new String[]{"0.1", "0.2", "0.3", "0.5", "0.8", "1"};
	ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, speeds);
	adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	ArrayAdapter<String> adapter2 = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, speeds);
	adapter2.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	s.setAdapter(adapter);
	s.setOnItemSelectedListener(this);
	s.setSelection(3);
	s2.setAdapter(adapter2);
	s2.setOnItemSelectedListener(this);
	s2.setSelection(3);
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
	  NodeConfiguration nodeConfiguration =
		        NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
		            getMasterUri());
	  
	  Log.e("masterUri", getMasterUri().toString());
	  mainExec = nodeMainExecutor;
	  nodeConfig = nodeConfiguration;
  }
  
  
  @Override
public boolean onKeyDown(int keyCode, KeyEvent event){

	  switch(keyCode){
	  case KeyEvent.KEYCODE_DPAD_LEFT:
		  twist.setX(-speed);
		  break;
	  case KeyEvent.KEYCODE_DPAD_RIGHT:
		  twist.setX(speed);
		  break;
	  case KeyEvent.KEYCODE_DPAD_UP:
		  twist.setY(speed);
		  break;
	  case KeyEvent.KEYCODE_DPAD_DOWN:
		  twist.setY(-speed);
		  break;
	  case KeyEvent.KEYCODE_A: //rechts
		  twist.setTheta(turnSpeed);
		  break;
	  case KeyEvent.KEYCODE_B:
		  twist.setTheta(-turnSpeed);
		  break;
	default: return false;
	  }
		publisher.publish(twist);

	  return false;
  }
  
  @Override
public boolean onKeyUp(int keyCode, KeyEvent event){

	  switch(keyCode){
	  case KeyEvent.KEYCODE_DPAD_LEFT:
		  twist.setY(0);
		  break;
	  case KeyEvent.KEYCODE_DPAD_RIGHT:
		  twist.setY(0);
		  break;
	  case KeyEvent.KEYCODE_DPAD_UP:
		  twist.setY(0);
		  break;
	  case KeyEvent.KEYCODE_DPAD_DOWN:
		  twist.setY(0);
		  break;
	  case KeyEvent.KEYCODE_A: //rechts
		  twist.setTheta(0);
		  break;
	  case KeyEvent.KEYCODE_B:
		  twist.setTheta(0);
		  break;
	  default: return false;
		  
	  }
		publisher.publish(twist);
	  return false;
  }  
}
