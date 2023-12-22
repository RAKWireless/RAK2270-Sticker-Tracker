// Main function to create BSP list
void binary_space_partition(uint16_t* src, uint16_t len,uint16_t exceed) 
{
  uint16_t dest[len];
  uint16_t dest_index = 0;
  uint32_t lower = len+1;
  uint32_t upper = len+1;
  for(int i = 0;i<len;i++)
  dest[i] = src[i];
  for(int i = 0;i<len;i++)
  {
    for(int j =0;j<len;j++)
    {
      if(dest[j]!= exceed && lower == len+1)
      lower = j;
      upper = j;
      if(dest[j] == exceed || j == len -1)
      {
        if(j==0)
          continue;
        if(dest[j] == exceed)
          upper = j-1;
        if(lower > upper)
          continue;
        src[dest_index] = dest[(upper + lower)/2];
        dest_index++;
        dest[(upper + lower)/2] = exceed;
        lower = len+1;
        upper = len+1;
      }    
  }
  lower = len+1;
  upper = len+1;
  if(dest_index > len)
  break;
  } 
  Serial.printf("binary_space_partion:");
  for(int i =0;i<len;i++)
  {
    Serial.printf("%u ",src[i]);
  }
  Serial.printf("\r\n");
  return ;
}
