/**
 * @file lv_event.h
 *
 */

#ifndef LV_EVENT_H
#define LV_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

struct _lv_obj_t;
struct _lv_event_dsc_t;

/**
 * Type of event being sent to the object.
 */
typedef enum {
    LV_EVENT_ALL = 0, /**< 所有事件 */

    /** 输入设备事件*/
    LV_EVENT_PRESSED,             /**< 对象已被按下 */
    LV_EVENT_PRESSING,            /**< 对象正在被按下（在按下时连续调用）*/
    LV_EVENT_PRESS_LOST,          /**< 对象仍然被按下，但光标/手指滑离对象 */
    LV_EVENT_SHORT_CLICKED,       /**< 对象被短时间按下，然后释放。如果发生滚动，不会调用此事件。*/
    LV_EVENT_LONG_PRESSED,        /**< 对象已经被按下至少 `long_press_time`。如果发生滚动，不会调用此事件。*/
    LV_EVENT_LONG_PRESSED_REPEAT, /**< 在每个 `long_press_repeat_time` 毫秒后调用。如果发生滚动，不会调用此事件。*/
    LV_EVENT_CLICKED,             /**< 如果没有滚动，则在释放时调用*/
    LV_EVENT_RELEASED,            /**< 在对象释放时的所有情况下调用*/
    LV_EVENT_SCROLL_BEGIN,        /**< 滚动开始。事件参数是指向滚动动画的指针，可以修改*/
    LV_EVENT_SCROLL_END,          /**< 滚动结束*/
    LV_EVENT_SCROLL,              /**< 滚动*/
    LV_EVENT_GESTURE,             /**< 检测到手势。使用 `lv_indev_get_gesture_dir(lv_indev_get_act());` 获取手势*/
    LV_EVENT_KEY,                 /**< 发送键到对象。使用 `lv_indev_get_key(lv_indev_get_act());` 获取键*/
    LV_EVENT_FOCUSED,             /**< 对象获取焦点*/
    LV_EVENT_DEFOCUSED,           /**< 对象失去焦点*/
    LV_EVENT_LEAVE,               /**< 对象失去焦点但仍被选中*/
    LV_EVENT_HIT_TEST,            /**< 执行高级命中测试*/

    /** 绘制事件*/
    LV_EVENT_COVER_CHECK,        /**< 检查对象是否完全覆盖一个区域。事件参数是 `lv_cover_check_info_t *`。*/
    LV_EVENT_REFR_EXT_DRAW_SIZE, /**< 获取对象周围所需的额外绘制区域的大小（例如阴影）。事件参数是 `lv_coord_t *` 以存储大小。*/
    LV_EVENT_DRAW_MAIN_BEGIN,    /**< 开始主绘制阶段*/
    LV_EVENT_DRAW_MAIN,          /**< 执行主绘制*/
    LV_EVENT_DRAW_MAIN_END,      /**< 完成主绘制阶段*/
    LV_EVENT_DRAW_POST_BEGIN,    /**< 开始后绘制阶段（当所有子项都绘制完毕时）*/
    LV_EVENT_DRAW_POST,          /**< 执行后绘制阶段（当所有子项都绘制完毕时）*/
    LV_EVENT_DRAW_POST_END,      /**< 完成后绘制阶段（当所有子项都绘制完毕时）*/
    LV_EVENT_DRAW_PART_BEGIN,    /**< 开始绘制部分。事件参数是 `lv_obj_draw_dsc_t *`。 */
    LV_EVENT_DRAW_PART_END,      /**< 完成绘制部分。事件参数是 `lv_obj_draw_dsc_t *`。 */

    /** 特殊事件*/
    LV_EVENT_VALUE_CHANGED,       /**< 对象的值已更改（例如滑块移动）*/
    LV_EVENT_INSERT,              /**< 有文本插入对象。事件数据是正在插入的 `char *`。*/
    LV_EVENT_REFRESH,             /**< 通知对象刷新一些内容（供用户使用）*/
    LV_EVENT_READY,               /**< 进程已完成*/
    LV_EVENT_CANCEL,              /**< 进程已取消 */

    /** 其他事件*/
    LV_EVENT_DELETE,              /**< 对象正在被删除*/
    LV_EVENT_CHILD_CHANGED,       /**< 子项已更改、添加或其大小、位置已更改 */
    LV_EVENT_CHILD_CREATED,       /**< 子项已创建，始终冒泡到所有父项*/
    LV_EVENT_CHILD_DELETED,       /**< 子项已删除，始终冒泡到所有父项*/
    LV_EVENT_SCREEN_UNLOAD_START, /**< 屏幕卸载已开始，在调用 scr_load 时立即触发*/
    LV_EVENT_SCREEN_LOAD_START,   /**< 屏幕加载已开始，在屏幕切换延迟过期时触发*/
    LV_EVENT_SCREEN_LOADED,       /**< 屏幕已加载*/
    LV_EVENT_SCREEN_UNLOADED,     /**< 屏幕已卸载*/
    LV_EVENT_SIZE_CHANGED,        /**< 对象的坐标/大小已更改*/
    LV_EVENT_STYLE_CHANGED,       /**< 对象的样式已更改*/
    LV_EVENT_LAYOUT_CHANGED,      /**< 由于布局重新计算，子项的位置已更改*/
    LV_EVENT_GET_SELF_SIZE,       /**< 获取小部件的内部大小*/

    _LV_EVENT_LAST,               /** 默认事件的数量*/


    LV_EVENT_PREPROCESS = 0x80,   /** 这是一个标志，可以与事件一起设置，以便在类默认事件处理之前处理 */
} lv_event_code_t;

typedef struct _lv_event_t {
    struct _lv_obj_t *target;
    struct _lv_obj_t *current_target;
    lv_event_code_t code;
    void *user_data;
    void *param;
    struct _lv_event_t *prev;
    uint8_t deleted: 1;
    uint8_t stop_processing: 1;
    uint8_t stop_bubbling: 1;
} lv_event_t;

/**
 * @brief Event callback.
 * Events are used to notify the user of some action being taken on the object.
 * For details, see ::lv_event_t.
 */
typedef void (*lv_event_cb_t)(lv_event_t *e);

/**
 * Used as the event parameter of ::LV_EVENT_HIT_TEST to check if an `point` can click the object or not.
 * `res` should be set like this:
 *   - If already set to `false` an other event wants that point non clickable. If you want to respect it leave it as `false` or set `true` to overwrite it.
 *   - If already set `true` and `point` shouldn't be clickable set to `false`
 *   - If already set to `true` you agree that `point` can click the object leave it as `true`
 */
typedef struct {
    const lv_point_t *point;   /**< A point relative to screen to check if it can click the object or not*/
    bool res;                   /**< true: `point` can click the object; false: it cannot*/
} lv_hit_test_info_t;

/**
 * Used as the event parameter of ::LV_EVENT_COVER_CHECK to check if an area is covered by the object or not.
 * In the event use `const lv_area_t * area = lv_event_get_cover_area(e)` to get the area to check
 * and `lv_event_set_cover_res(e, res)` to set the result.
 */
typedef struct {
    lv_cover_res_t res;
    const lv_area_t *area;
} lv_cover_check_info_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Send an event to the object
 * @param obj           pointer to an object
 * @param event_code    the type of the event from `lv_event_t`
 * @param param         arbitrary data depending on the widget type and the event. (Usually `NULL`)
 * @return LV_RES_OK: `obj` was not deleted in the event; LV_RES_INV: `obj` was deleted in the event_code
 */
lv_res_t lv_event_send(struct _lv_obj_t *obj, lv_event_code_t event_code, void *param);

/**
 * Used by the widgets internally to call the ancestor widget types's event handler
 * @param class_p   pointer to the class of the widget (NOT the ancestor class)
 * @param e         pointer to the event descriptor
 * @return          LV_RES_OK: the target object was not deleted in the event; LV_RES_INV: it was deleted in the event_code
 */
lv_res_t lv_obj_event_base(const lv_obj_class_t *class_p, lv_event_t *e);

/**
 * Get the object originally targeted by the event. It's the same even if the event is bubbled.
 * @param e     pointer to the event descriptor
 * @return      the target of the event_code
 */
struct _lv_obj_t *lv_event_get_target(lv_event_t *e);

/**
 * Get the current target of the event. It's the object which event handler being called.
 * If the event is not bubbled it's the same as "normal" target.
 * @param e     pointer to the event descriptor
 * @return      pointer to the current target of the event_code
 */
struct _lv_obj_t *lv_event_get_current_target(lv_event_t *e);

/**
 * Get the event code of an event
 * @param e     pointer to the event descriptor
 * @return      the event code. (E.g. `LV_EVENT_CLICKED`, `LV_EVENT_FOCUSED`, etc)
 */
lv_event_code_t lv_event_get_code(lv_event_t *e);

/**
 * Get the parameter passed when the event was sent
 * @param e     pointer to the event descriptor
 * @return      pointer to the parameter
 */
void *lv_event_get_param(lv_event_t *e);

/**
 * Get the user_data passed when the event was registered on the object
 * @param e     pointer to the event descriptor
 * @return      pointer to the user_data
 */
void *lv_event_get_user_data(lv_event_t *e);

/**
 * Stop the event from bubbling.
 * This is only valid when called in the middle of an event processing chain.
 * @param e     pointer to the event descriptor
 */
void lv_event_stop_bubbling(lv_event_t *e);

/**
 * Stop processing this event.
 * This is only valid when called in the middle of an event processing chain.
 * @param e     pointer to the event descriptor
 */
void lv_event_stop_processing(lv_event_t *e);

/**
 * Register a new, custom event ID.
 * It can be used the same way as e.g. `LV_EVENT_CLICKED` to send custom events
 * @return      the new event id
 * @example
 * uint32_t LV_EVENT_MINE = 0;
 * ...
 * e = lv_event_register_id();
 * ...
 * lv_event_send(obj, LV_EVENT_MINE, &some_data);
 */
uint32_t lv_event_register_id(void);

/**
 * Nested events can be called and one of them might belong to an object that is being deleted.
 * Mark this object's `event_temp_data` deleted to know that its `lv_event_send` should return `LV_RES_INV`
 * @param obj pointer to an object to mark as deleted
 */
void _lv_event_mark_deleted(struct _lv_obj_t *obj);


/**
 * Add an event handler function for an object.
 * Used by the user to react on event which happens with the object.
 * An object can have multiple event handler. They will be called in the same order as they were added.
 * @param obj       pointer to an object
 * @param filter    and event code (e.g. `LV_EVENT_CLICKED`) on which the event should be called. `LV_EVENT_ALL` can be sued the receive all the events.
 * @param event_cb  the new event function
 * @param           user_data custom data data will be available in `event_cb`
 * @return          a pointer the event descriptor. Can be used in ::lv_obj_remove_event_dsc
 */
struct _lv_event_dsc_t *lv_obj_add_event_cb(struct _lv_obj_t *obj, lv_event_cb_t event_cb, lv_event_code_t filter,
                                            void *user_data);

/**
 * Remove an event handler function for an object.
 * @param obj       pointer to an object
 * @param event_cb  the event function to remove, or `NULL` to remove the firstly added event callback
 * @return          true if any event handlers were removed
 */
bool lv_obj_remove_event_cb(struct _lv_obj_t *obj, lv_event_cb_t event_cb);

/**
 * Remove an event handler function with a specific user_data from an object.
 * @param obj               pointer to an object
 * @param event_cb          the event function to remove, or `NULL` only `user_data` matters.
 * @param event_user_data   the user_data specified in ::lv_obj_add_event_cb
 * @return                  true if any event handlers were removed
 */
bool lv_obj_remove_event_cb_with_user_data(struct _lv_obj_t *obj, lv_event_cb_t event_cb,
                                           const void *event_user_data);

/**
 * DEPRECATED because doesn't work if multiple event handlers are added to an object.
 * Remove an event handler function for an object.
 * @param obj       pointer to an object
 * @param event_dsc pointer to an event descriptor to remove (returned by ::lv_obj_add_event_cb)
 * @return          true if any event handlers were removed
 */
bool lv_obj_remove_event_dsc(struct _lv_obj_t *obj, struct _lv_event_dsc_t *event_dsc);

/**
 * The user data of an event object event callback. Always the first match with `event_cb` will be returned.
 * @param obj               pointer to an object
 * @param event_cb          the event function
 * @return                  the user_data
 */
void *lv_obj_get_event_user_data(struct _lv_obj_t *obj, lv_event_cb_t event_cb);

/**
 * Get the input device passed as parameter to indev related events.
 * @param e     pointer to an event
 * @return      the indev that triggered the event or NULL if called on a not indev related event
 */
lv_indev_t *lv_event_get_indev(lv_event_t *e);

/**
 * Get the part draw descriptor passed as parameter to `LV_EVENT_DRAW_PART_BEGIN/END`.
 * @param e     pointer to an event
 * @return      the part draw descriptor to hook the drawing or NULL if called on an unrelated event
 */
lv_obj_draw_part_dsc_t *lv_event_get_draw_part_dsc(lv_event_t *e);

/**
 * Get the draw context which should be the first parameter of the draw functions.
 * Namely: `LV_EVENT_DRAW_MAIN/POST`, `LV_EVENT_DRAW_MAIN/POST_BEGIN`, `LV_EVENT_DRAW_MAIN/POST_END`
 * @param e     pointer to an event
 * @return      pointer to a draw context or NULL if called on an unrelated event
 */
lv_draw_ctx_t *lv_event_get_draw_ctx(lv_event_t *e);

/**
 * Get the old area of the object before its size was changed. Can be used in `LV_EVENT_SIZE_CHANGED`
 * @param e     pointer to an event
 * @return      the old absolute area of the object or NULL if called on an unrelated event
 */
const lv_area_t *lv_event_get_old_size(lv_event_t *e);

/**
 * Get the key passed as parameter to an event. Can be used in `LV_EVENT_KEY`
 * @param e     pointer to an event
 * @return      the triggering key or NULL if called on an unrelated event
 */
uint32_t lv_event_get_key(lv_event_t *e);

/**
 * Get the animation descriptor of a scrolling. Can be used in `LV_EVENT_SCROLL_BEGIN`
 * @param e     pointer to an event
 * @return      the animation that will scroll the object. (can be modified as required)
 */
lv_anim_t *lv_event_get_scroll_anim(lv_event_t *e);

/**
 * Set the new extra draw size. Can be used in `LV_EVENT_REFR_EXT_DRAW_SIZE`
 * @param e     pointer to an event
 * @param size  The new extra draw size
 */
void lv_event_set_ext_draw_size(lv_event_t *e, lv_coord_t size);

/**
 * Get a pointer to an `lv_point_t` variable in which the self size should be saved (width in `point->x` and height `point->y`).
 * Can be used in `LV_EVENT_GET_SELF_SIZE`
 * @param e     pointer to an event
 * @return      pointer to `lv_point_t` or NULL if called on an unrelated event
 */
lv_point_t *lv_event_get_self_size_info(lv_event_t *e);

/**
 * Get a pointer to an `lv_hit_test_info_t` variable in which the hit test result should be saved. Can be used in `LV_EVENT_HIT_TEST`
 * @param e     pointer to an event
 * @return      pointer to `lv_hit_test_info_t` or NULL if called on an unrelated event
 */
lv_hit_test_info_t *lv_event_get_hit_test_info(lv_event_t *e);

/**
 * Get a pointer to an area which should be examined whether the object fully covers it or not.
 * Can be used in `LV_EVENT_HIT_TEST`
 * @param e     pointer to an event
 * @return      an area with absolute coordinates to check
 */
const lv_area_t *lv_event_get_cover_area(lv_event_t *e);

/**
 * Set the result of cover checking. Can be used in `LV_EVENT_COVER_CHECK`
 * @param e     pointer to an event
 * @param res   an element of ::lv_cover_check_info_t
 */
void lv_event_set_cover_res(lv_event_t *e, lv_cover_res_t res);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_EVENT_H*/
