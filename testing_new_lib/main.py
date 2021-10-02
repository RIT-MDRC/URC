from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      translation_vector=[-10, 0, 5],
      orientation=[0, 1.57, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      translation_vector=[25, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      translation_vector=[22, 0, 0],
      orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])