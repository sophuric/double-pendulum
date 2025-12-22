// helper macros for doubly linked lists
// works with any struct which contains the fields 'prev' and 'next' as pointers to the struct type
#define LL_INSERT_BEFORE(item, first, last, insert_before)                                                                                             \
	{                                                                                                                                                  \
		if (insert_before) {                                                                                                                           \
			if ((insert_before)->prev) (insert_before)->prev->next = (item); /* update the next pointer of the previous one before insert_before */    \
			if ((insert_before) == (first)) (first) = (item);                /* if we are inserting before the first one, update the new first item */ \
			(item)->prev = (insert_before)->prev;                            /* set pointers for new item */                                           \
			(item)->next = (insert_before);                                                                                                            \
			(insert_before)->prev = (item); /* update the previous pointer of insert_before */                                                         \
		} else                                                                                                                                         \
			LL_ADD(item, first, last)                                                                                                                  \
	}
#define LL_ADD(item, first, last)                                                               \
	{                                                                                           \
		if (last) {                      /* add to end */                                       \
			(item)->next = (last)->next; /* last->next is most likely NULL, but just in case */ \
			if ((last)->next) (last)->next->prev = (item);                                      \
			(last)->next = (item);                                                              \
			(item)->prev = (last); /* set pointers for new item */                              \
			(last) = (item);       /* set new last item */                                      \
		} else {                   /* adding first one */                                       \
			/* asserting first is unset*/                                                       \
			(first) = (item);                                                                   \
			(last) = (item);                                                                    \
			(item)->prev = 0;                                                                   \
			(item)->next = 0;                                                                   \
		}                                                                                       \
	}
#define LL_REMOVE(item, first, last)                                                                             \
	{                                                                                                            \
		if ((item)->prev) (item)->prev->next = (item)->next; /* link previous one to next one */                 \
		if ((item)->next) (item)->next->prev = (item)->prev; /* link next one back to previous one */            \
		if ((first) == (item)) (first) = (item)->next;       /* update first if we are removing the first one */ \
		if ((last) == (item)) (last) = (item)->prev;         /* update last if we are removing the last one */   \
		(item)->next = 0;                                                                                        \
		(item)->prev = 0;                                                                                        \
	}
#define LL_REMOVE_ALL(first, remove_var, remove_code) \
	{                                                 \
		while (first) {                               \
			(remove_var) = (first);                   \
			(first) = (first)->next;                  \
			remove_code;                              \
		}                                             \
	}
#define LL_REMOVE_ALL_TYPEOF(first, remove_var, remove_code) LL_REMOVE_ALL(first, typeof(first) remove_var, remove_code)
#define LL_LOOP(loop_type, loop_var, first) for (loop_type loop_var = first; loop_var; loop_var = loop_var->next)
#define LL_LOOP_TYPEOF(loop_var, first) LL_LOOP_TYPE(typeof(first), loop_var, first)
