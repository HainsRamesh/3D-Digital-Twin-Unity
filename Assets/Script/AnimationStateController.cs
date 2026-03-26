using System;
using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
    public float walkSpeed = 3f;
    public float runSpeed = 6f;

    private Animator anim;
    private float currentSpeed;

    int Blend;

    void Start()
    {
        anim = GetComponent<Animator>();

        Blend = Animator.StringToHash("Speed");
       
    }

    void Update()
    {
        float targetSpeed = 0f;
        float moveSpeed = 0f;

        bool walkPressed = Input.GetKey(KeyCode.W);
        bool runPressed = Input.GetKey(KeyCode.LeftShift);

        if (walkPressed)
        {
            if (runPressed)
            {
                targetSpeed = 1f;      
                moveSpeed = runSpeed;
            }
            else
            {
                targetSpeed = 0.5f;   
                moveSpeed = walkSpeed;
            }

            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
        }
        if (!walkPressed && !runPressed)
        {
            currentSpeed = Mathf.Lerp(currentSpeed, 0f, Time.deltaTime * 10f); ;
        }


            currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.deltaTime * 10f);
        anim.SetFloat(Blend, currentSpeed);
    }
}
