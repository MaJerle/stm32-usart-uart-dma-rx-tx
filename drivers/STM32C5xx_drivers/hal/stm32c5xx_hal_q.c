/**
  **********************************************************************************************************************
  * @file    stm32c5xx_hal_q.c
  * @brief   This file provides Q services.
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "stm32_hal.h"

/**
  * @addtogroup STM32C5xx_HAL_Driver
  * @{
  */
#if (defined(USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1))

/** @addtogroup Q
  * @{
  */

/** @defgroup Q_Introduction Q Introduction
  * @{

Q handles linked-list operations.

A queue is a set of linked nodes, each containing data and a link to the next node.

The following operations are supported: initialize and de-initialize the queue, and insert, remove,
 or replace nodes at the head, tail, or any position.

A linked list can be made circular, looping back to any position in the queue.

One queue can also be inserted into another, which merges them.

Create nodes externally using drivers such as DMA, SD, or MMC,
using functions like HAL_DMA_FillNodeConfig.

  */
/**
  * @}
  */

/** @defgroup Q_How_To_Use Q How To Use
  * @{

# How to use the Q HAL module driver

## The Q HAL module can be used as follows:

Q is the abbreviation for Queue. It is an entity that contains a node or a set of nodes linked to each other.
Each node contains a data set and a link to the next node.

Use this utility HAL driver only with HAL modules that support the linked-list feature.
This module is activated automatically when the USE_HAL_PPP_LINKEDLIST compilation flag is enabled in
stm32ynxx_hal_conf.h.
Use this module to build a linked-list Q executable for masters that support the linked-list feature.
To build Q(s) compatible with different linked-list masters, this module supports two addressing modes:
   - Direct addressing mode: node link addresses represent the physical node address.
   - Base offset addressing mode: node link addresses represent the offset of the node from the Q head node
     address.
This module supports singly linked-list Q nodes.
Behavior is not guaranteed if a Q is modified outside this module.

This module provides six API sets that allow you to:

1. Initialize and de-initialize the logical Q object:
   - Initialize the logical Q object using information provided by any HAL peripheral module that supports the linked
     list feature. When initialized, the Q is ready to apply any operation provided by this module.
   - De-initialize the logical Q object and unlink all Q node(s). When de-initialized, reuse the Q object for the same
     or another master executor.

2. Insert a new node in a Q:
   - Insert a new node into a Q at any position using the following function models:
     - Generic new node insertion function that allows you to add a new node at any selected Q position.
       - This functionality is ensured by HAL_Q_InsertNode() function. The p_node parameter allows you to specify where
         the new node is inserted.
         - When p_node is null, the new node is placed at the head of Q.
         - When p_node is not null, the new node is placed directly after p_node. If p_node is not found within the Q,
           the function returns an error and the Q is not modified.
     - New head node insertion function that allows you to add a new node as the head node of a Q.
       - This functionality is ensured by HAL_Q_InsertNode_Head() function.
     - New tail node insertion function that allows you to add a new node as the tail node of a Q.
       - This functionality is ensured by HAL_Q_InsertNode_Tail() function.
   - Do not add a new node when the selected Q is circular.
   - Prefer the tail node insertion model function to reduce footprint.

3. Remove an existing node from a Q:
   - Remove any existing node from a Q using the following function models:
     - Generic node removal function that allows you to remove any existing node from a Q.
       - This functionality is ensured by HAL_Q_RemoveNode() function.
         - The p_node parameter allows you to select the node to be removed. This function returns an error when p_node
           is not found within the Q or p_node is null.
     - Head node removal function that allows you to remove the head node from a Q.
       - This functionality is ensured by HAL_Q_RemoveNode_Head() function.
     - Tail node removal function that allows you to remove the tail node from a Q.
       - This functionality is ensured by HAL_Q_RemoveNode_Tail() function.
   - Do not remove an existing node when the selected Q is circular.
   - When successfully removed, the removed node can be reused later.

4. Replace an existing node in a Q:
   - Replace any existing node with a new node in a Q using the following function models:
     - Generic node replacement function that allows you to replace any existing node with a new node in a Q.
       - This functionality is ensured by HAL_Q_ReplaceNode() function. The p_old_node parameter allows you to specify
         the node to be replaced.
         - When p_old_node is not null, p_new_node replaces p_old_node. If p_old_node is not found within the Q, the
           function returns an error and the Q is not modified.
     - Head node replacement function that allows you to replace the existing head node with a new head node in a Q.
       - This functionality is ensured by HAL_Q_ReplaceNode_Head() function.
     - Tail node replacement function that allows you to replace the existing tail node with a new tail node in a Q.
       - This functionality is ensured by HAL_Q_ReplaceNode_Tail() function.
   - Do not replace an existing node when the selected Q is circular.
   - When successfully replaced, the replaced node can be reused later.

5. Insert a source Q into a destination Q:
   - Insert a source Q into a destination Q at any position using the following function models:
     - Generic source Q insertion function that allows you to insert source Q node(s) at any selected destination Q
       position.
       - This functionality is ensured by HAL_Q_InsertNode() function. The p_node parameter allows you to specify where
         the source Q node(s) are inserted.
         - When p_node is null, the source Q node(s) are placed at the head of the destination Q.
         - When p_node is not null, the source Q node(s) are placed directly after p_node. If p_node is not found
           within the destination Q, the function returns an error and the source and destination Qs are not modified.
     - Head source Q insertion function that allows you to insert source Q node(s) before all the destination Q node(s).
       - This functionality is ensured by HAL_Q_InsertNode_Head() function.
     - Tail source Q insertion function that allows you to insert source Q node(s) after all the destination Q node(s).
       - This functionality is ensured by HAL_Q_InsertNode_Tail() function.
   - Do not add source Q node(s) to destination Q node(s) when any Q is circular.
   - Prefer the tail Q insertion model function to reduce footprint.
   - When successfully inserted, the destination Q contains the source Q nodes and destination Q nodes, and the source Q
     is cleared and can be reused later without needing to reinitialize it.

6. Set and clear a circular link on a non-empty Q:
   - Set a circular link to any Q node using the following function models:
     - Generic circular link Q setting function that allows you to set a circular link at any non-empty Q position.
       - This functionality is ensured by HAL_Q_SetCircularLinkQ() function. The p_node parameter allows you to specify
         the first circular node (node linked to Q tail node). This function returns an error when p_node is not found
         within the Q or p_node is null.
     - Head circular link Q setting function that allows you to set a circular link to the head node of Q.
       - This functionality is ensured by HAL_Q_SetCircularLinkQ_Head() function.
     - Tail circular link Q setting function that allows you to set a circular link to the tail node of Q.
       - This functionality is ensured by HAL_Q_SetCircularLinkQ_Tail() function.
   - Clear a circular link from a Q.
     - This functionality is ensured by HAL_Q_ClearCircularLinkQ() function.
  */
/**
  * @}
  */

/** @defgroup Q_Configuration_Table Q Configuration Table
  * @{
## Configuration inside the Q module

Config definitions       | Description     | Default value | Note
------------------------ | --------------- | ------------- | ---------------------------------------------------------
USE_ASSERT_DBG_PARAM     | from IDE        |     None      | When defined, enable parameter asserts.
USE_HAL_CHECK_PARAM      | from hal_conf.h |     0U        | It allows using run-time checks on parameters.
USE_HAL_{PPP}_LINKEDLIST | from hal_conf.h |     0U        | It allows using the PPP in linked-list mode.
USE_HAL_Q_CIRCULAR_LINK  | from hal_ppp.h  |     0U        | It allows using a circular-link queue.
  */
/**
  * @}
  */

/* Private constants -------------------------------------------------------------------------------------------------*/
/* Private types -----------------------------------------------------------------------------------------------------*/
/* Private functions -------------------------------------------------------------------------------------------------*/
/** @defgroup Q_Private_Functions Q Private Functions
  * @{
  */
static void Q_UnlinkNodes(hal_q_t *p_q, uint32_t head_node_addr);
static void Q_ResetInfo(hal_q_t *p_q);
static hal_status_t Q_FindNode(const hal_q_t *p_q, uint32_t head_node_addr, uint32_t node_addr,
                               uint32_t *p_prev_node_addr);
/**
  * @}
  */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/** @addtogroup Q_Exported_Functions
  * @{
  */

/** @addtogroup Q_Exported_Functions_Group1 Q Initialization and de-initialization functions
  * @{
This section provides functions to initialize and de-initialize the logical Q object:
- Call the function HAL_Q_Init() to initialize the logical Q object and associate its operation information.
- Call the function HAL_Q_DeInit() to de-initialize the logical Q object and unlink its nodes.
  */

/**
  * @brief  Initialize the logical Q object and associate its operation information.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_desc_ops        Pointer to a hal_q_desc_ops_t structure that contains Q operation information.
  * @note                     The p_desc_ops is a constant provided by HAL PPP modules that support the linked-list
  *                           feature, titled HAL_PPP_{mode}_DescOps or HAL_PPP_DescOps.
  * @retval HAL_OK            In the case of a valid initialization.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_Init(hal_q_t *p_q, const hal_q_desc_ops_t *p_desc_ops)
{
  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_desc_ops != NULL);

#if defined(USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_q == NULL) || (p_desc_ops == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  p_desc_ops->p_get_node_info(&p_q->next_addr_offset, &p_q->addr_mode);

  p_q->p_head_node           = NULL;
  p_q->p_tail_node           = NULL;
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  p_q->p_first_circular_node = NULL;
#endif /* USE_HAL_Q_CIRCULAR_LINK */
  p_q->node_nbr              = 0U;
  p_q->p_set_node            = p_desc_ops->p_set_node;
  p_q->p_get_node            = p_desc_ops->p_get_node;

  return HAL_OK;
}

/**
  * @brief De-initialize the logical Q object and unlink its node(s).
  * @param p_q Pointer to a hal_q_t structure that contains Q information.
  */
void HAL_Q_DeInit(hal_q_t *p_q)
{
  uint32_t head;

  ASSERT_DBG_PARAM(p_q != NULL);

  head = (uint32_t)p_q->p_head_node;

  Q_UnlinkNodes(p_q, head);

  p_q->p_head_node = NULL;
}
/**
  * @}
  */

/** @addtogroup Q_Exported_Functions_Group2 Q node insertion functions
  * @{
This section provides functions to insert a new node at any Q position:
- Call the function HAL_Q_InsertNode() to insert a new node in any Q position.
- Call the function HAL_Q_InsertNode_Head() to insert a new node in the head position of a Q.
- Call the function HAL_Q_InsertNode_Tail() to insert a new node in the tail position of a Q.
  */

/**
  * @brief  Insert a new node after a previous node in a Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_node            Pointer to the node that specifies the insertion position. When null, the new node is
  *                           placed at the head of Q.
  * @param  p_new_node        Pointer to a new node.
  * @retval HAL_OK            In the case of a successful node insertion.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  * @retval HAL_ERROR         In the case of the p_prev_node not found in the Q.
  */
hal_status_t HAL_Q_InsertNode(hal_q_t *p_q, const void *p_node, void *p_new_node)
{
  uint32_t head;
  uint32_t node;
  uint32_t new_node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_new_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  node     = (uint32_t)p_node;
  new_node = (uint32_t)p_new_node;
  offset   = p_q->next_addr_offset;

  /* Empty Q */
  if ((p_q->p_head_node == NULL) && (p_node == NULL))
  {
    p_q->p_head_node = p_new_node;
    p_q->p_tail_node = p_new_node;
  }
  /* Non-empty Q */
  else if (p_q->p_head_node != NULL)
  {
    /* Insert node at head level */
    if (p_node == NULL)
    {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
      if (p_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
      {
        p_q->p_set_node(head, new_node, head, offset);
      }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

      p_q->p_head_node = p_new_node;
    }
    else
    {
      /* Insert node at tail level */
      if (p_node == p_q->p_tail_node)
      {
        p_q->p_set_node(head, node, new_node, offset);
        p_q->p_tail_node = p_new_node;
      }
      /* Insert node at middle level */
      else
      {
        /* Find node */
        if (Q_FindNode(p_q, head, node, NULL) == HAL_OK)
        {
          p_q->p_set_node(head, new_node, p_q->p_get_node(head, node, offset), offset);
          p_q->p_set_node(head, node, new_node, offset);
        }
        else
        {
          return HAL_ERROR;
        }
      }
    }
  }
  else
  {
    return HAL_ERROR;
  }

  p_q->node_nbr++;

  return HAL_OK;
}

/**
  * @brief Insert a new node at the head of the Q.
  * @param p_q                Pointer to a hal_q_t structure that contains Q information.
  * @param p_new_node         Pointer to a new node.
  * @retval HAL_OK            In the case of a successful node insertion at the head of the Q.
  * @retval HAL_ERROR         In the case of a node insertion failure.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_InsertNode_Head(hal_q_t *p_q, void *p_new_node)
{
  uint32_t head;
  uint32_t new_node;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  uint32_t offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_new_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  new_node = (uint32_t)p_new_node;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  offset   = p_q->next_addr_offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  /* Empty Q */
  if (p_q->p_head_node == NULL)
  {
    p_q->p_head_node = p_new_node;
    p_q->p_tail_node = p_new_node;
  }
  /* Non-empty Q */
  else
  {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
    if (p_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
    {
      p_q->p_set_node(head, new_node, head, offset);
    }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

    p_q->p_head_node = p_new_node;
  }

  p_q->node_nbr++;

  return HAL_OK;
}

/**
  * @brief  Insert a new node at the tail of the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_new_node        Pointer to a new node.
  * @retval HAL_OK            In the case of a successful node insertion at the tail of the Q.
  * @retval HAL_ERROR         In the case of a node insertion failure.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_InsertNode_Tail(hal_q_t *p_q, void *p_new_node)
{
  uint32_t head;
  uint32_t tail;
  uint32_t new_node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_new_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  tail     = (uint32_t)p_q->p_tail_node;
  new_node = (uint32_t)p_new_node;
  offset   = p_q->next_addr_offset;

  /* Empty Q */
  if (p_q->p_head_node == NULL)
  {
    p_q->p_head_node = p_new_node;
    p_q->p_tail_node = p_new_node;
  }
  /* Non-empty Q */
  else
  {
    p_q->p_set_node(head, tail, new_node, offset);
    p_q->p_tail_node = p_new_node;
  }

  p_q->node_nbr++;

  return HAL_OK;
}
/**
  * @}
  */


/** @addtogroup Q_Exported_Functions_Group3 Q node removing functions
  * @{
This section provides functions to remove any existing node from a Q:
- Call the function HAL_Q_RemoveNode() to remove any existing node from a Q.
- Call the function HAL_Q_RemoveNode_Head() to remove the head node from a Q.
- Call the function HAL_Q_RemoveNode_Tail() to remove the tail node from a Q.
  */

/**
  * @brief  Remove a node from the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_node            Pointer to the previous node.
  * @retval HAL_OK            In the case of a successful node removal.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  * @retval HAL_ERROR         In the case of a node not found in the Q.
  */
hal_status_t HAL_Q_RemoveNode(hal_q_t *p_q, const void *p_node)
{
  uint32_t prev;
  uint32_t head;
  uint32_t tail;
  uint32_t node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head   = (uint32_t)p_q->p_head_node;
  tail   = (uint32_t)p_q->p_tail_node;
  node   = (uint32_t)p_node;
  offset = p_q->next_addr_offset;

  /* Delete head node */
  if (p_node == p_q->p_head_node)
  {
    if (p_q->node_nbr == 1U)
    {
      Q_ResetInfo(p_q);
    }
    else
    {
      /* Set the new head node */
      p_q->p_head_node = (void *)(p_q->p_get_node(head, head, offset));
      p_q->p_set_node(0U, node, 0U, offset);
    }
  }
  else
  {
    /* Delete tail node */
    if (p_node == p_q->p_tail_node)
    {
      prev = tail;
    }
    /* Delete middle node */
    else
    {
      prev = node;
    }

    if (Q_FindNode(p_q, head, prev, &prev) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Delete tail node */
    if (p_node == p_q->p_tail_node)
    {
      /* Set the new tail node */
      p_q->p_tail_node = (void *)prev;
      p_q->p_set_node(0U, prev, 0U, offset);
    }
    /* Delete middle node */
    else
    {
      p_q->p_set_node(head, prev, p_q->p_get_node(head, node, offset), offset);
      p_q->p_set_node(0U, node, 0U, offset);
    }
  }

  p_q->node_nbr--;

  return HAL_OK;
}

/**
  * @brief  Remove the head node of the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of a successful head node removal.
  * @retval HAL_ERROR         In the case of a node not found.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_RemoveNode_Head(hal_q_t *p_q)
{
  uint32_t head;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_head_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  offset = p_q->next_addr_offset;

  if (p_q->node_nbr == 1U)
  {
    Q_ResetInfo(p_q);
  }
  else
  {
    /* Store the old head node */
    head = (uint32_t)p_q->p_head_node;

    /* Set the new head node */
    p_q->p_head_node = (void *)(p_q->p_get_node(head, head, offset));
    p_q->p_set_node(0U, head, 0U, offset);
  }

  p_q->node_nbr--;

  return HAL_OK;
}

/**
  * @brief  Remove the tail node of the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of a successful tail node removal.
  * @retval HAL_ERROR         In the case of a node not found.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_RemoveNode_Tail(hal_q_t *p_q)
{
  uint32_t prev = 0U;
  uint32_t head;
  uint32_t tail;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_tail_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_tail_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head   = (uint32_t)p_q->p_head_node;
  tail   = (uint32_t)p_q->p_tail_node;
  offset = p_q->next_addr_offset;

  if (p_q->node_nbr == 1U)
  {
    Q_ResetInfo(p_q);
  }
  else
  {
    if (Q_FindNode(p_q, head, tail, &prev) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Set the new tail node */
    p_q->p_tail_node = (void *)prev;
    p_q->p_set_node(0U, prev, 0U, offset);
  }

  p_q->node_nbr--;

  return HAL_OK;
}
/**
  * @}
  */


/** @addtogroup Q_Exported_Functions_Group4 Q node replacing functions
  * @{
This section provides functions to replace any existing node in a Q:
- Call the function HAL_Q_ReplaceNode() to replace any existing node with a new node in a Q.
- Call the function HAL_Q_ReplaceNode_Head() to replace the head node in a Q.
- Call the function HAL_Q_ReplaceNode_Tail() to replace the tail node in a Q.
  */

/**
  * @brief  Replace a node in the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_old_node        Pointer to an old node.
  * @param  p_new_node        Pointer to a new node.
  * @retval HAL_OK            In the case of a successful node replacement.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  * @retval HAL_ERROR         In the case of the old node not found in the Q.
  */
hal_status_t HAL_Q_ReplaceNode(hal_q_t *p_q, const void *p_old_node, void *p_new_node)
{
  uint32_t prev;
  uint32_t head;
  uint32_t tail;
  uint32_t new_node;
  uint32_t old_node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_old_node != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_old_node == NULL) || (p_new_node == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  tail     = (uint32_t)p_q->p_tail_node;
  new_node = (uint32_t)p_new_node;
  old_node = (uint32_t)p_old_node;
  offset   = p_q->next_addr_offset;

  if (p_old_node == p_q->p_head_node)
  {
    if (p_q->node_nbr == 1U)
    {
      p_q->p_tail_node = p_new_node;
    }
    else
    {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
      if (p_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
      {
        p_q->p_set_node(head, new_node, p_q->p_get_node(head, head, offset), offset);
      }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

      p_q->p_set_node(0U, head, 0U, offset);
    }

    p_q->p_head_node = p_new_node;
  }
  else
  {
    if (p_old_node == p_q->p_tail_node)
    {
      prev = tail;
    }
    else
    {
      prev = old_node;
    }

    if (Q_FindNode(p_q, head, prev, &prev) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (p_old_node == p_q->p_tail_node)
    {
      p_q->p_set_node(head, prev, new_node, offset);
      p_q->p_tail_node = p_new_node;
    }
    else
    {
      p_q->p_set_node(head, new_node, p_q->p_get_node(head, old_node, offset), offset);
      p_q->p_set_node(head, prev, new_node, offset);
      p_q->p_set_node(0U, old_node, 0U, offset);
    }
  }

  return HAL_OK;
}

/**
  * @brief  Replace the head node in the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_new_node        Pointer to a new node.
  * @retval HAL_OK            In the case of a successful head node replacement.
  * @retval HAL_ERROR         In the case of the old node not found in the Q.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_ReplaceNode_Head(hal_q_t *p_q, void *p_new_node)
{
  uint32_t head;
  uint32_t new_node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_q->p_head_node == NULL) || (p_new_node == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  new_node = (uint32_t)p_new_node;
  offset   = p_q->next_addr_offset;

  if (p_q->node_nbr == 1U)
  {
    p_q->p_tail_node = p_new_node;
  }
  else
  {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
    if (p_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
    {
      p_q->p_set_node(head, new_node, p_q->p_get_node(head, head, offset), offset);
    }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

    p_q->p_set_node(0U, head, 0U, offset);
  }

  p_q->p_head_node = p_new_node;

  return HAL_OK;
}

/**
  * @brief  Replace the tail node in the Q.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_new_node        Pointer to a new node.
  * @retval HAL_OK            In the case of a successful tail node replacement.
  * @retval HAL_ERROR         In the case of the old node not found in the Q.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_ReplaceNode_Tail(hal_q_t *p_q, void *p_new_node)
{
  uint32_t prev = 0U;
  uint32_t head;
  uint32_t tail;
  uint32_t new_node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_new_node != NULL);
  ASSERT_DBG_PARAM(p_q->p_tail_node != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if ((p_q->p_tail_node == NULL) || (p_new_node == NULL))
  {
    return HAL_INVALID_PARAM;
  }
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if (p_q->p_first_circular_node != NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  head     = (uint32_t)p_q->p_head_node;
  tail     = (uint32_t)p_q->p_tail_node;
  new_node = (uint32_t)p_new_node;
  offset   = p_q->next_addr_offset;

  if (p_q->node_nbr == 1U)
  {
    p_q->p_head_node = p_new_node;
  }
  else
  {
    /* Find the tail previous node */
    if (Q_FindNode(p_q, head, tail, &prev) != HAL_OK)
    {
      return HAL_ERROR;
    }

    p_q->p_set_node(head, prev, new_node, offset);
  }

  p_q->p_tail_node = p_new_node;

  return HAL_OK;
}
/**
  * @}
  */


/** @addtogroup Q_Exported_Functions_Group5 Q inserting Q functions
  * @{
This section provides functions to insert a source Q at any destination Q position:
- Call the function HAL_Q_InsertQ() to insert a source Q at any destination Q position.
- Call the function HAL_Q_InsertQ_Head() to insert a source Q before all destination Q node(s).
- Call the function HAL_Q_InsertQ_Tail() to insert a source Q after all destination Q node(s).
  */

/**
  * @brief  Insert a source Q directly after the previous node in the destination Q.
  * @param  p_dest_q Pointer to a hal_q_t structure that contains Q information.
  * @param  p_src_q  Pointer to a hal_q_t structure that contains Q information.
  * @param  p_node   Pointer to the previous node. When null, the source Q node(s) are placed at the head of the
  *                 destination Q.
  * @retval HAL_OK            In the case of a successful source Q insertion in the destination Q.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  * @retval HAL_ERROR         In the case of the p_prev_node not found in the Q.
  */
hal_status_t HAL_Q_InsertQ(hal_q_t *p_dest_q, hal_q_t *p_src_q, const void *p_node)
{
  uint32_t src_head;
  uint32_t dest_head;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  uint32_t src_tail;
  uint32_t dest_tail;
  uint32_t offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */
  uint32_t node_addr;

  ASSERT_DBG_PARAM(p_src_q != NULL);
  ASSERT_DBG_PARAM(p_dest_q != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_src_q->p_first_circular_node == NULL);
  ASSERT_DBG_PARAM(p_dest_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if ((p_src_q->p_first_circular_node != NULL) || (p_dest_q->p_first_circular_node != NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  src_head  = (uint32_t)p_src_q->p_head_node;
  dest_head = (uint32_t)p_dest_q->p_head_node;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  src_tail  = (uint32_t)p_src_q->p_tail_node;
  dest_tail = (uint32_t)p_dest_q->p_tail_node;
  offset    = p_dest_q->next_addr_offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */
  node_addr = (uint32_t)p_node;

  /* Empty source Q */
  if (p_src_q->node_nbr == 0U)
  {
    return HAL_OK;
  }

  /* Empty destination Q */
  if (p_dest_q->p_head_node == NULL)
  {
    p_dest_q->p_head_node = p_src_q->p_head_node;
    p_dest_q->p_tail_node = p_src_q->p_tail_node;
  }
  /* Non-empty destination Q */
  else
  {
    /* Insert source Q at head level of destination Q */
    if (p_node == NULL)
    {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
      if (p_dest_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
      {
        /* Link source Q tail node address to destination Q head node address */
        p_src_q->p_set_node(src_head, src_tail, dest_head, offset);
      }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

      /* Set destination Q head node as source Q head node */
      p_dest_q->p_head_node = p_src_q->p_head_node;
    }
    else
    {
      if (Q_FindNode(p_dest_q, dest_head, node_addr, NULL) != HAL_OK)
      {
        return HAL_ERROR;
      }

#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
      if (p_dest_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
      {
        if (p_node == p_dest_q->p_tail_node)
        {
          /* Link source Q to tail destination Q */
          p_src_q->p_set_node(dest_head, dest_tail, src_head, offset);
        }
        else
        {
          /* Link source Q to middle destination Q */
          p_dest_q->p_set_node(dest_head, src_tail, p_dest_q->p_get_node(dest_head, node_addr, offset), offset);
          p_dest_q->p_set_node(dest_head, node_addr, src_head, offset);
        }
      }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

      if (p_node == p_dest_q->p_tail_node)
      {
        /* Set source Q tail node as destination Q tail node */
        p_dest_q->p_tail_node = p_src_q->p_tail_node;
      }
    }
  }

  /* Set destination Q node number */
  p_dest_q->node_nbr += p_src_q->node_nbr;

  Q_ResetInfo(p_src_q);
  p_src_q->node_nbr = 0U;

  return HAL_OK;
}

/**
  * @brief  Insert a source Q at the head of the destination Q.
  * @param  p_dest_q          Pointer to a hal_q_t structure that contains Q information.
  * @param  p_src_q           Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of a successful source Q insertion at the head of the destination Q.
  * @retval HAL_ERROR         In the case of the source Q not inserted in the destination Q.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_InsertQ_Head(hal_q_t *p_dest_q, hal_q_t *p_src_q)
{
  uint32_t src_head;
  uint32_t dest_head;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  uint32_t src_tail;
  uint32_t offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  ASSERT_DBG_PARAM(p_src_q != NULL);
  ASSERT_DBG_PARAM(p_dest_q != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_src_q->p_first_circular_node == NULL);
  ASSERT_DBG_PARAM(p_dest_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if ((p_src_q->p_first_circular_node != NULL) || (p_dest_q->p_first_circular_node != NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  src_head  = (uint32_t)p_src_q->p_head_node;
  dest_head = (uint32_t)p_dest_q->p_head_node;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  src_tail  = (uint32_t)p_src_q->p_tail_node;
  offset    = p_dest_q->next_addr_offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  if (p_src_q->node_nbr == 0U)
  {
    return HAL_OK;
  }

  /* Empty destination Q */
  if (p_dest_q->p_head_node == NULL)
  {
    p_dest_q->p_head_node = p_src_q->p_head_node;
    p_dest_q->p_tail_node = p_src_q->p_tail_node;
  }
  /* Non-empty destination Q */
  else
  {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
    if (p_dest_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
    {
      /* Link source Q tail node address to destination Q head node address */
      p_src_q->p_set_node(src_head, src_tail, dest_head, offset);
    }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

    /* Set destination Q head node as source Q head node */
    p_dest_q->p_head_node = p_src_q->p_head_node;
  }

  /* Set node number of new Q */
  p_dest_q->node_nbr += p_src_q->node_nbr;

  Q_ResetInfo(p_src_q);
  p_src_q->node_nbr = 0U;

  return HAL_OK;
}

/**
  * @brief  Insert a source Q at the tail of the destination Q.
  * @param  p_dest_q          Pointer to a hal_q_t structure that contains Q information.
  * @param  p_src_q           Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of a successful source Q insertion at the tail of the destination Q.
  * @retval HAL_ERROR         In the case of the source Q not inserted in the destination Q.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_InsertQ_Tail(hal_q_t *p_dest_q, hal_q_t *p_src_q)
{
  uint32_t src_head_addr;
  uint32_t dest_head_addr;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  uint32_t dest_tail_addr;
  uint32_t offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  ASSERT_DBG_PARAM(p_src_q != NULL);
  ASSERT_DBG_PARAM(p_dest_q != NULL);
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  ASSERT_DBG_PARAM(p_src_q->p_first_circular_node == NULL);
  ASSERT_DBG_PARAM(p_dest_q->p_first_circular_node == NULL);
#endif /* USE_HAL_Q_CIRCULAR_LINK */

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  if ((p_src_q->p_first_circular_node != NULL) || (p_dest_q->p_first_circular_node != NULL))
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_Q_CIRCULAR_LINK */
#endif /* USE_HAL_CHECK_PARAM */

  src_head_addr  = (uint32_t)p_src_q->p_head_node;
  dest_head_addr = (uint32_t)p_dest_q->p_head_node;
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
  dest_tail_addr = (uint32_t)p_dest_q->p_tail_node;
  offset         = p_dest_q->next_addr_offset;
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

  /* Check if source Q is empty */
  if (p_src_q->node_nbr == 0U)
  {
    return HAL_OK;
  }

  /* Empty destination Q */
  if (p_dest_q->p_head_node == NULL)
  {
    p_dest_q->p_head_node   = p_src_q->p_head_node;
    p_dest_q->p_tail_node   = p_src_q->p_tail_node;
  }
  /* Non-empty destination Q */
  else
  {
#if (USE_HAL_Q_DIRECT_ADDR_MODE) && (USE_HAL_Q_DIRECT_ADDR_MODE == 1)
    if (p_dest_q->addr_mode == HAL_Q_ADDRESSING_DIRECT)
    {
      /* Link source Q tail node address to destination Q head node address */
      p_src_q->p_set_node(dest_head_addr, dest_tail_addr, src_head_addr, offset);
    }
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

    /* Set destination Q tail node as source Q tail node */
    p_dest_q->p_tail_node = p_src_q->p_tail_node;
  }

  /* Set node number of new Q */
  p_dest_q->node_nbr += p_src_q->node_nbr;

  Q_ResetInfo(p_src_q);
  p_src_q->node_nbr = 0U;

  return HAL_OK;
}
/**
  * @}
  */

#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
/** @addtogroup Q_Exported_Functions_Group6 Q circularizing Q functions
  * @{
This section provides functions to set and clear a circular link to any node in a non-empty Q:
- Call the function HAL_Q_SetCircularLinkQ() to set a circular link to any node in a non-empty Q.
- Call the function HAL_Q_SetCircularLinkQ_Head() to set a circular link to the head Q node.
- Call the function HAL_Q_SetCircularLinkQ_Tail() to set a circular link to the tail Q node.
- Call the function HAL_Q_ClearCircularLinkQ() to clear any existing circular link in a Q.
  */

/**
  * @brief  Set a circular link to any selected Q node.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @param  p_node            Pointer to the first circular node.
  * @retval HAL_OK            In the case of Q circularization.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  * @retval HAL_ERROR         In case of p_node node not found in the Q.
  */
hal_status_t HAL_Q_SetCircularLinkQ(hal_q_t *p_q, void *p_node)
{
  uint32_t head;
  uint32_t tail;
  uint32_t node;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_head_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  head   = (uint32_t)p_q->p_head_node;
  tail   = (uint32_t)p_q->p_tail_node;
  node   = (uint32_t)p_node;
  offset = p_q->next_addr_offset;

  if (Q_FindNode(p_q, head, node, NULL) == HAL_OK)
  {
    /* Link the tail node to the p_node */
    p_q->p_set_node(head, tail, node, offset);

    /* Update first circular node in Q */
    p_q->p_first_circular_node = p_node;
  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief Set Q circular mode.
  * @param p_q                Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of Q circularized on the head.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_SetCircularLinkQ_Head(hal_q_t *p_q)
{
  uint32_t head;
  uint32_t tail;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_head_node == NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  head   = (uint32_t)p_q->p_head_node;
  tail   = (uint32_t)p_q->p_tail_node;
  offset = p_q->next_addr_offset;

  /* Link the tail node to the head node */
  p_q->p_set_node(head, tail, head, offset);

  /* Update first circular node in Q */
  p_q->p_first_circular_node = p_q->p_head_node;

  return HAL_OK;
}

/**
  * @brief  Set Q circular mode.
  * @param  p_q               Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK            In the case of Q circularized on the tail.
  * @retval HAL_INVALID_PARAM In the case of an invalid parameter.
  */
hal_status_t HAL_Q_SetCircularLinkQ_Tail(hal_q_t *p_q)
{
  uint32_t head;
  uint32_t tail;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_tail_node ==  NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  head   = (uint32_t)p_q->p_head_node;
  tail   = (uint32_t)p_q->p_tail_node;
  offset = p_q->next_addr_offset;

  /* Link the tail node to itself */
  p_q->p_set_node(head, tail, tail, offset);

  /* Update first circular node in Q */
  p_q->p_first_circular_node = p_q->p_tail_node;

  return HAL_OK;
}

/**
  * @brief Clear circular mode of the Q.
  * @param p_q     Pointer to a hal_q_t structure that contains Q information.
  * @retval HAL_OK In the case of Q being cleared.
  */
hal_status_t HAL_Q_ClearCircularLinkQ(hal_q_t *p_q)
{
  uint32_t tail;
  uint32_t offset;

  ASSERT_DBG_PARAM(p_q != NULL);
  ASSERT_DBG_PARAM(p_q->p_head_node != NULL);

#if defined (USE_HAL_CHECK_PARAM) && (USE_HAL_CHECK_PARAM == 1)
  if (p_q->p_tail_node ==  NULL)
  {
    return HAL_INVALID_PARAM;
  }
#endif /* USE_HAL_CHECK_PARAM */

  tail   = (uint32_t)p_q->p_tail_node;
  offset = p_q->next_addr_offset;

  /* Clear circular link within tail node */
  p_q->p_set_node(0U, tail, 0U, offset);

  /* Update first circular node in Q */
  p_q->p_first_circular_node = NULL;

  return HAL_OK;
}
#endif /* USE_HAL_Q_CIRCULAR_LINK */
/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup Q_Private_Functions
  * @{
  */

/**
  * @brief Unlink all Q nodes.
  * @param p_q            Pointer to a hal_q_t structure that contains Q information.
  * @param head_node_addr Head node address.
  */
static void Q_UnlinkNodes(hal_q_t *p_q, uint32_t head_node_addr)
{
  uint32_t current_node = head_node_addr;
  uint32_t offset       = p_q->next_addr_offset;
  uint32_t next_node;

  /* Repeat for all Q nodes */
  while (p_q->node_nbr > 0U)
  {
    next_node = p_q->p_get_node(head_node_addr, current_node, offset);
    p_q->p_set_node(0U, current_node, 0U, offset);
    current_node = next_node;
    p_q->node_nbr--;
  }
}

/**
  * @brief Reset Q information.
  * @param p_q Pointer to a hal_q_t structure that contains Q information.
  */
static void Q_ResetInfo(hal_q_t *p_q)
{
  p_q->p_head_node           = NULL;
  p_q->p_tail_node           = NULL;
#if defined(USE_HAL_Q_CIRCULAR_LINK) && (USE_HAL_Q_CIRCULAR_LINK == 1)
  p_q->p_first_circular_node = NULL;
#endif /* USE_HAL_Q_CIRCULAR_LINK */
}

/**
  * @brief Find a node in the Q.
  * @param p_q              Pointer to a hal_q_t structure that contains Q information.
  * @param head_node_addr   Head node address.
  * @param node_addr        Node address to find.
  * @param p_prev_node_addr Pointer to the previous node address.
  * @retval HAL_OK          In the case of a found node.
  * @retval HAL_ERROR       In the case of the node not found in the Q.
  */
static hal_status_t Q_FindNode(const hal_q_t *p_q, uint32_t head_node_addr, uint32_t node_addr,
                               uint32_t *p_prev_node_addr)
{
  uint32_t current_node_addr = head_node_addr;
  uint32_t offset            = p_q->next_addr_offset;
  uint32_t current_node_idx  = 0U;

  /* Loop while node not found */
  while ((current_node_addr != node_addr) && (current_node_idx < p_q->node_nbr))
  {
    if (p_prev_node_addr != NULL)
    {
      *p_prev_node_addr = current_node_addr;
    }

    current_node_addr = p_q->p_get_node(head_node_addr, current_node_addr, offset);
    current_node_idx++;
  }

  if ((current_node_idx == p_q->node_nbr) && (current_node_addr != node_addr))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* USE_HAL_Q_DIRECT_ADDR_MODE */

/**
  * @}
  */
