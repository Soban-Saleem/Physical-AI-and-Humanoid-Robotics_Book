#!/bin/bash
# Script Name: generate_api_endpoint.sh
# Description: Generates a FastAPI endpoint with models and documentation
# Usage: ./generate_api_endpoint.sh [endpoint_name] [model_name]

# Validate input arguments
if [ $# -ne 2 ]; then
    echo "Usage: $0 [endpoint_name] [model_name]"
    echo "Example: $0 'get_user' 'User'"
    exit 1
fi

ENDPOINT_NAME="$1"
MODEL_NAME="$2"

# Generate the API endpoint file
cat << 'EOF' > "${ENDPOINT_NAME}_endpoint.py"
from typing import Optional
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter()

# Request/Response Model
class ${MODEL_NAME}Base(BaseModel):
    name: str
    description: Optional[str] = None

class ${MODEL_NAME}Create(${MODEL_NAME}Base):
    pass

class ${MODEL_NAME}(${MODEL_NAME}Base):
    id: int

# Sample in-memory storage (replace with database in real implementation)
items = []

@router.get("/${ENDPOINT_NAME}/{item_id}", response_model=${MODEL_NAME})
async def get_${ENDPOINT_NAME}(item_id: int):
    '''
    Get a specific ${ENDPOINT_NAME} by ID.
    '''
    for item in items:
        if item.id == item_id:
            return item
    raise HTTPException(status_code=404, detail="${MODEL_NAME} not found")

@router.post("/${ENDPOINT_NAME}/", response_model=${MODEL_NAME})
async def create_${ENDPOINT_NAME}(item: ${MODEL_NAME}Create):
    '''
    Create a new ${ENDPOINT_NAME}.
    '''
    new_item = ${MODEL_NAME}(
        id=len(items) + 1,
        name=item.name,
        description=item.description
    )
    items.append(new_item)
    return new_item

@router.put("/${ENDPOINT_NAME}/{item_id}", response_model=${MODEL_NAME})
async def update_${ENDPOINT_NAME}(item_id: int, item: ${MODEL_NAME}Create):
    '''
    Update an existing ${ENDPOINT_NAME}.
    '''
    for i, existing_item in enumerate(items):
        if existing_item.id == item_id:
            updated_item = ${MODEL_NAME}(
                id=item_id,
                name=item.name,
                description=item.description
            )
            items[i] = updated_item
            return updated_item
    raise HTTPException(status_code=404, detail="${MODEL_NAME} not found")

@router.delete("/${ENDPOINT_NAME}/{item_id}")
async def delete_${ENDPOINT_NAME}(item_id: int):
    '''
    Delete a ${ENDPOINT_NAME} by ID.
    '''
    global items
    items = [item for item in items if item.id != item_id]
    return {"message": "${MODEL_NAME} deleted successfully"}
EOF

echo "API endpoint for '$ENDPOINT_NAME' with model '$MODEL_NAME' generated successfully!"
echo "File created: ${ENDPOINT_NAME}_endpoint.py"
echo ""
echo "To use this endpoint, import and include it in your main FastAPI application:"
echo "from $ENDPOINT_NAME\_endpoint import router as $ENDPOINT_NAME\_router"
echo "app.include_router($ENDPOINT_NAME\_router, prefix="/api/v1")"