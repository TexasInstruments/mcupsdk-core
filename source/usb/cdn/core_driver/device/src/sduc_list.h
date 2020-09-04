/******************************************************************************
*
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
* sduc_list.h
* Functions for creating and management circular list
******************************************************************************/

#ifndef CDN_LIST_H
#define CDN_LIST_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "list_if.h"
#include "list_structs_if.h"

/*Function initialize list*/
static inline void listInit(LIST_ListHead *list) {
    list->next = list;
    list->prev = list;
}

/*Add new element to the list between prev and next*/
static inline void listAddElement(LIST_ListHead * item, LIST_ListHead * prev, LIST_ListHead * next) {
    next->prev = item;
    item->next = next;
    item->prev = prev;
    prev->next = item;

}

/*Add new entry after a specified head*/
static inline void listAdd(LIST_ListHead * item, LIST_ListHead * head) {
    listAddElement(item, head, head->next);
}
/*Add new entry to the end of the list*/
static inline void listAddTail(LIST_ListHead * item, LIST_ListHead *head) {
    listAddElement(item, head->prev, head);
}

/* delete item from list */
static inline void listDelete(LIST_ListHead * list) {
    list->next->prev = list->prev;
    list->prev->next = list->next;

    list->prev = NULL;
    list->next = NULL;
}


#ifdef __cplusplus
}
#endif

#endif
