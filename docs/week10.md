# Week 10: Model Training & Deployment

## Learning Objectives

By the end of this week, you will be able to:
- Prepare datasets for training
- Train imitation learning models
- Evaluate model performance
- Deploy trained policies on physical robots

## Prerequisites

- Completed Week 9 (50+ recorded episodes)
- Google account (for Colab)

---

## Part 1: Preparing Your Dataset

### 1.1 Dataset Structure

Your recorded episodes should be in:
```
recorded_episodes/
├── episode_0000.npz
├── episode_0001.npz
├── episode_0002.npz
...
└── episode_0049.npz
```

### 1.2 Dataset Validation

Before training, verify your data:

```python
from pathlib import Path
import numpy as np

data_dir = Path("recorded_episodes")
episodes = sorted(data_dir.glob("episode_*.npz"))

print(f"Total episodes: {len(episodes)}")

# Check each episode
for ep_file in episodes[:5]:  # Sample first 5
    data = np.load(ep_file)
    print(f"{ep_file.name}:")
    print(f"  Frames: {len(data['timestamp'])}")
    print(f"  Duration: {data['timestamp'][-1]:.2f}s")
    print(f"  Obs shape: {data['observation.state'].shape}")
    print(f"  Act shape: {data['action'].shape}")
```

### 1.3 Upload to HuggingFace

```python
from huggingface_hub import HfApi

api = HfApi()

# Create dataset repo
api.create_repo(
    repo_id="your-username/so101-pick-place",
    repo_type="dataset",
    private=True
)

# Upload files
api.upload_folder(
    folder_path="recorded_episodes",
    repo_id="your-username/so101-pick-place",
    repo_type="dataset"
)
```

---

## Part 2: Training in Google Colab

### 2.1 Setup Notebook

Create a new Colab notebook and run:

```python
# Install dependencies
!pip install lerobot torch transformers datasets

# Check GPU
import torch
print(f"GPU available: {torch.cuda.is_available()}")
print(f"GPU name: {torch.cuda.get_device_name(0)}")
```

### 2.2 Load Dataset

```python
from datasets import load_dataset

# Load your dataset from HuggingFace
dataset = load_dataset("your-username/so101-pick-place")

print(f"Train examples: {len(dataset['train'])}")
```

### 2.3 Configure Training

```python
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy

# Model configuration
config = ACTConfig(
    input_shapes={
        "observation.state": [6],  # 6 joint positions
    },
    output_shapes={
        "action": [6],  # 6 joint commands
    },
    # Architecture
    dim_model=256,
    n_heads=8,
    n_encoder_layers=4,
    n_decoder_layers=1,
    # Training
    chunk_size=100,  # Predict 100 timesteps
)

# Create policy
policy = ACTPolicy(config)
print(f"Parameters: {sum(p.numel() for p in policy.parameters()):,}")
```

### 2.4 Training Loop

```python
from torch.utils.data import DataLoader
import torch.optim as optim

# Create dataloader
train_loader = DataLoader(
    dataset['train'],
    batch_size=32,
    shuffle=True
)

# Optimizer
optimizer = optim.AdamW(policy.parameters(), lr=1e-4)

# Training
num_epochs = 100
for epoch in range(num_epochs):
    total_loss = 0
    for batch in train_loader:
        optimizer.zero_grad()

        # Forward pass
        loss = policy.compute_loss(batch)

        # Backward pass
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    avg_loss = total_loss / len(train_loader)
    if epoch % 10 == 0:
        print(f"Epoch {epoch}: Loss = {avg_loss:.4f}")

# Save model
torch.save(policy.state_dict(), "policy_checkpoint.pt")
```

### 2.5 Download Trained Model

```python
from google.colab import files

# Download the checkpoint
files.download("policy_checkpoint.pt")
```

---

## Part 3: Policy Inference

### 3.1 Load Trained Model

```python
import torch
import numpy as np

# Load model architecture
policy = ACTPolicy(config)

# Load trained weights
policy.load_state_dict(torch.load("policy_checkpoint.pt"))
policy.eval()
```

### 3.2 Predict Action

```python
def predict_action(observation: np.ndarray) -> np.ndarray:
    """
    Run model inference.

    Args:
        observation: Current joint positions (6,)

    Returns:
        Predicted action (6,) joint targets
    """
    with torch.no_grad():
        # Prepare input
        obs_tensor = torch.tensor(observation, dtype=torch.float32)
        obs_tensor = obs_tensor.unsqueeze(0)  # Add batch dim

        # Predict
        action = policy.select_action({"observation.state": obs_tensor})

        # Convert back to numpy
        return action.squeeze(0).numpy()
```

### 3.3 Inference Node

```python
class PolicyInferenceNode(Node):
    def __init__(self):
        super().__init__('policy_inference')

        # Load model
        self.policy = self.load_policy("policy_checkpoint.pt")

        # Subscribe to current state
        self.state_sub = self.create_subscription(
            JointState,
            '/so101_follower/joint_states',
            self.state_callback,
            10
        )

        # Publish actions
        self.action_pub = self.create_publisher(
            JointState,
            '/so101_follower/joint_commands',
            10
        )

        # Inference timer (10 Hz)
        self.timer = self.create_timer(0.1, self.inference_loop)

        self.current_state = None

    def state_callback(self, msg):
        self.current_state = np.array(msg.position)

    def inference_loop(self):
        if self.current_state is None:
            return

        # Predict action
        action = self.predict_action(self.current_state)

        # Publish command
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['1', '2', '3', '4', '5', '6']
        cmd.position = action.tolist()
        self.action_pub.publish(cmd)
```

---

## Part 4: Evaluation

### 4.1 Success Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| Success Rate | Tasks completed / attempts | > 50% |
| Completion Time | Seconds to complete task | < 10s |
| Smoothness | Jerk (rate of acceleration change) | Low |
| Consistency | Variance across attempts | Low |

### 4.2 Evaluation Protocol

```
For 10 test episodes:
1. Place cube at position A
2. Start policy inference
3. Wait for completion (max 15 seconds)
4. Record:
   - Success (cube at B): Yes/No
   - Time to complete
   - Any issues observed
5. Reset and repeat
```

### 4.3 Recording Results

```python
results = []

for trial in range(10):
    result = {
        "trial": trial,
        "success": input("Success? (y/n): ").lower() == 'y',
        "time": float(input("Time (seconds): ")),
        "notes": input("Notes: ")
    }
    results.append(result)

# Calculate metrics
successes = sum(1 for r in results if r['success'])
success_rate = successes / len(results)
avg_time = np.mean([r['time'] for r in results if r['success']])

print(f"\nResults:")
print(f"  Success Rate: {success_rate*100:.0f}%")
print(f"  Avg Time (success): {avg_time:.1f}s")
```

---

## Part 5: Debugging Failures

### 5.1 Common Issues

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| Robot doesn't move | Model not loaded | Check checkpoint path |
| Wrong movements | Poor training data | Record more/better demos |
| Jerky motion | Low inference rate | Increase timer frequency |
| Overshooting | Action scaling wrong | Check normalization |
| Inconsistent | Not enough data | Record more episodes |

### 5.2 Improving Performance

1. **More Data:** Record 50 more episodes
2. **Data Augmentation:** Add noise to training
3. **Longer Training:** Run more epochs
4. **Architecture:** Try different model sizes
5. **Hyperparameters:** Tune learning rate, batch size

### 5.3 A/B Testing

```python
# Compare two policies
policy_a = load_policy("checkpoint_v1.pt")
policy_b = load_policy("checkpoint_v2.pt")

# Run same test on both
results_a = run_evaluation(policy_a, num_trials=10)
results_b = run_evaluation(policy_b, num_trials=10)

print(f"Policy A success rate: {results_a['success_rate']:.0%}")
print(f"Policy B success rate: {results_b['success_rate']:.0%}")
```

---

## Lab Exercise

### Task 1: Train Your Model

1. Upload dataset to HuggingFace
2. Create Colab notebook
3. Train for 100 epochs
4. Download checkpoint

### Task 2: Deploy on Robot

1. Load trained model in ROS2 node
2. Run inference loop
3. Test on physical robot

### Task 3: Evaluate Performance

1. Run 10 test trials
2. Record success/failure
3. Calculate success rate
4. Document failure modes

### Task 4: Improve (Optional)

1. Record 20 more demonstrations
2. Retrain model
3. Compare before/after performance

### Capstone Deliverables

- [ ] Trained model checkpoint
- [ ] Evaluation results (10 trials)
- [ ] Written report including:
  - Data collection process
  - Training curves
  - Success rate and analysis
  - Failure mode documentation
  - Ideas for improvement
- [ ] Video of robot executing learned task

---

## Report Template

```markdown
# SO-101 Imitation Learning Report

## 1. Data Collection
- Total episodes recorded: __
- Task description: __
- Average episode length: __ seconds

## 2. Training
- Model architecture: ACT
- Training epochs: __
- Final training loss: __
- Training time: __ minutes

## 3. Evaluation
- Test trials: 10
- Successful: __/10
- Success rate: __%
- Average completion time: __ seconds

## 4. Failure Analysis
Observed failure modes:
1. __
2. __

## 5. Conclusions
What worked well:
- __

What could be improved:
- __

## 6. Future Work
- __
```

---

## What's Next?

Congratulations on completing the course! You've learned:
- ROS2 robot control
- Serial communication
- Web interfaces
- Embedded systems
- Imitation learning

Continue exploring:
- More complex tasks (stacking, sorting)
- Vision-based policies (add camera)
- Multi-task learning
- Real-world applications

---

## Reference

### LeRobot Resources

- Documentation: https://huggingface.co/docs/lerobot
- GitHub: https://github.com/huggingface/lerobot
- Model Hub: https://huggingface.co/models?library=lerobot

### Training Tips

```python
# Learning rate schedule
scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
    optimizer,
    T_max=num_epochs,
    eta_min=1e-6
)

# Gradient clipping
torch.nn.utils.clip_grad_norm_(policy.parameters(), max_norm=1.0)

# Mixed precision training
scaler = torch.cuda.amp.GradScaler()
with torch.cuda.amp.autocast():
    loss = policy.compute_loss(batch)
```
