# Possible Override Types
# Do not apply contact margin data
uint8 NONE=0
# Replace the contact manager's CollisionMarginData
uint8 REPLACE=1
# Override the contact managers default margin only
uint8 OVERRIDE_DEFAULT_MARGIN=2
# Override the contact managers pair margin only. This does not preserve any existing pair margin data
uint8 OVERRIDE_PAIR_MARGIN=3
# Modify the contact managers pair margin only.
# This will preserve existing pairs not being modified by the provided margin data.
# If a pair already exist it will be updated with the provided margin data.
uint8 MODIFY_PAIR_MARGIN=4

# one of the option above
uint8 type
