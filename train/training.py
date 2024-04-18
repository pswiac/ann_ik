import pandas as pd
from sklearn.model_selection import train_test_split
from tensorflow import keras
from keras import layers

import numpy as np
import ast

# Load the dataset
df = pd.read_csv('data/robotic_arm_solutions.csv')

# Convert string representations of lists to actual lists
df['Joint_Angles'] = df['Joint_Angles'].apply(ast.literal_eval)
df['Task_Space_Position'] = df['Task_Space_Position'].apply(ast.literal_eval)

# Convert to numpy arrays
joint_angles = np.array(df['Joint_Angles'].tolist())
task_space_positions = np.array(df['Task_Space_Position'].tolist())

# Split the dataset into training and testing sets, keeping 'Seed' with the test set
X_train, X_test, y_train, y_test = train_test_split(task_space_positions, joint_angles, test_size=0.2, random_state=42)

# Neural network model
model = keras.Sequential([
    layers.Dense(64, activation='relu', input_shape=(X_train.shape[1],)),  # Minus 1 for 'Seed' column
    layers.Dense(64, activation='relu'),
    layers.Dense(y_train.shape[1])  # Output layer neurons equal to the number of target features
])

# Model summary to check the architecture
model.summary()

model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])

# Train the model (excluding 'Seed' from training data)
model.fit(X_train, y_train, epochs=80, batch_size=32, validation_split=0.2)


# print(X_test.shape)
# Make predictions
predictions = model.predict(X_test)
single_sample = X_test[1][np.newaxis, :] 
prediction_single = model.predict(single_sample)


# Create a DataFrame from predictions
predictions_df = pd.DataFrame(predictions, columns=['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6', 'Joint_7'])  # Adjust column names as needed

# Export predictions to a CSV file
predictions_df.to_csv('data/model_predictions.csv', index=False)

print("Predictions saved to data/model_predictions.csv")

# Evaluate the model on the test set
loss, accuracy = model.evaluate(X_test, y_test)
print(accuracy)

# Save the trained model
model.save('../simulation/ann_model')  
print("Model saved to '../ur5_robot_kinematics/ann_model'")

